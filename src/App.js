import React, { useState, useEffect, useCallback } from 'react';

const ChargePathPlanner = () => {
  const GRID_SIZE = 40;
  const CELL_SIZE = 15;
  const REPULSION_STRENGTH = 2.5;
  const REPULSION_RADIUS = 6;
  const MAX_ITERATIONS = 500; // Safety limit, but we run until solved,change this for a complex obstacle terrain 
  const PERTURBATION_STRENGTH = 1.5; // This is the strength given when the intersection has equal left and right force
  const FORCE_BALANCE_THRESHOLD = 0.3; // Consider forces balanced if below this

  const [obstacles, setObstacles] = useState([
    { type: 'rectangle', x1: 12, y1: 8, x2: 18, y2: 16 },
    { type: 'circle', cx: 25, cy: 22, radius: 4 },
    { type: 'triangle', points: [{x: 8, y: 24}, {x: 14, y: 24}, {x: 11, y: 30}] },
  ]);
  
  const [start, setStart] = useState({ x: 2, y: 2 });
  const [goal, setGoal] = useState({ x: 37, y: 37 });
  const [pathPoints, setPathPoints] = useState([]);
  const [currentStep, setCurrentStep] = useState(0);
  const [isAnimating, setIsAnimating] = useState(false);
  const [history, setHistory] = useState([]);
  const [intersections, setIntersections] = useState([]);
  const [forceVectors, setForceVectors] = useState([]);
  const [editMode, setEditMode] = useState(null);
  const [tempObstacle, setTempObstacle] = useState(null);
  const [algorithmPhase, setAlgorithmPhase] = useState('idle');
  const [stepMessage, setStepMessage] = useState('');
  const [selectedShape, setSelectedShape] = useState('rectangle');
  const [inflationRadius, setInflationRadius] = useState(1.5);
  const [trianglePoints, setTrianglePoints] = useState([]);
  const [arrowScale, setArrowScale] = useState(3);
  const [stepSize, setStepSize] = useState(0.5);
  const [enableSmoothing, setEnableSmoothing] = useState(false);
  const [smoothingIterations, setSmoothingIterations] = useState(3);
  const animationIntervalRef = React.useRef(null);

  // Smooth path using Chaikin's algorithm (corner cutting)
  const smoothPathPoints = useCallback((points, iterations) => {
    if (!points || points.length < 3) return points;
    
    let smoothed = [...points];
    
    for (let iter = 0; iter < iterations; iter++) {
      const newPoints = [];
      
      // Always keep the first point (start)
      newPoints.push(smoothed[0]);
      
      for (let i = 0; i < smoothed.length - 1; i++) {
        const p0 = smoothed[i];
        const p1 = smoothed[i + 1];
        
        if (i === 0) {
          const q = {
            ...p0,
            x: p0.x * 0.75 + p1.x * 0.25,
            y: p0.y * 0.75 + p1.y * 0.25
          };
          const r = {
            ...p1,
            x: p0.x * 0.25 + p1.x * 0.75,
            y: p0.y * 0.25 + p1.y * 0.75
          };
          newPoints.push(q, r);
        } else if (i === smoothed.length - 2) {
          const q = {
            ...p0,
            x: p0.x * 0.75 + p1.x * 0.25,
            y: p0.y * 0.75 + p1.y * 0.25
          };
          newPoints.push(q);
        } else {
          // Middle segments: standard Chaikin (25% and 75%)
          const q = {
            ...p0,
            x: p0.x * 0.75 + p1.x * 0.25,
            y: p0.y * 0.75 + p1.y * 0.25
          };
          const r = {
            ...p1,
            x: p0.x * 0.25 + p1.x * 0.75,
            y: p0.y * 0.25 + p1.y * 0.75
          };
          newPoints.push(q, r);
        }
      }
      
      newPoints.push(smoothed[smoothed.length - 1]);
      
      smoothed = newPoints;
    }
    
    return smoothed;
  }, []);

  // Get display points (smoothed or original)
  const getDisplayPoints = useCallback((points) => {
    if (!enableSmoothing || smoothingIterations === 0) {
      return points;
    }
    return smoothPathPoints(points, smoothingIterations);
  }, [enableSmoothing, smoothingIterations, smoothPathPoints]);

  // Check if point is inside any obstacle shape (with optional inflation) or outside map bounds
  const isPointInObstacle = useCallback((x, y, obstacle, useInflation = true) => {
    const inflation = useInflation ? inflationRadius : 0;
    
    // Special case: check if it's the wall/boundary obstacle
    if (obstacle.type === 'wall') {
      // Point is "inside wall" if it's outside the map bounds (with inflation inward)
      return x < inflation || x > GRID_SIZE - 1 - inflation || 
             y < inflation || y > GRID_SIZE - 1 - inflation;
    }
    
    if (obstacle.type === 'rectangle' || (!obstacle.type && obstacle.x1 !== undefined)) {
      // Rectangle
      const x1 = obstacle.x1 - inflation;
      const y1 = obstacle.y1 - inflation;
      const x2 = obstacle.x2 + inflation;
      const y2 = obstacle.y2 + inflation;
      return x >= x1 && x <= x2 && y >= y1 && y <= y2;
    } 
    else if (obstacle.type === 'circle') {
      // Circle
      const dx = x - obstacle.cx;
      const dy = y - obstacle.cy;
      const distance = Math.sqrt(dx * dx + dy * dy);
      return distance <= obstacle.radius + inflation;
    } 
    else if (obstacle.type === 'triangle') {
      // Triangle - use barycentric coordinates with inflation
      const p = obstacle.points;
      
      // First check if inside the triangle
      const area = (p[1].x - p[0].x) * (p[2].y - p[0].y) - (p[2].x - p[0].x) * (p[1].y - p[0].y);
      const s = ((p[1].x - p[0].x) * (y - p[0].y) - (p[1].y - p[0].y) * (x - p[0].x)) / area;
      const t = ((p[2].x - p[0].x) * (y - p[0].y) - (p[2].y - p[0].y) * (x - p[0].x)) / area;
      
      const insideTriangle = s >= 0 && t >= 0 && (1 - s - t) >= 0;
      if (insideTriangle) return true;
      
      // Check inflation zone
      if (inflation > 0) {
        for (let i = 0; i < 3; i++) {
          const p1 = p[i];
          const p2 = p[(i + 1) % 3];
          const dist = pointToLineDistance(x, y, p1.x, p1.y, p2.x, p2.y);
          if (dist <= inflation) {
            const segLen = Math.sqrt((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2);
            const dot = ((x - p1.x) * (p2.x - p1.x) + (y - p1.y) * (p2.y - p1.y)) / (segLen * segLen);
            if (dot >= 0 && dot <= 1) return true;
          }
          const distToVertex = Math.sqrt((x - p1.x) ** 2 + (y - p1.y) ** 2);
          if (distToVertex <= inflation) return true;
        }
      }
      
      return false;
    }
    
    return false;
  }, [inflationRadius]);

  // Wall obstacle - represents the map boundaries
  const wallObstacle = { type: 'wall' };

  // Helper: distance from point to line segment
  const pointToLineDistance = (px, py, x1, y1, x2, y2) => {
    const A = px - x1;
    const B = py - y1;
    const C = x2 - x1;
    const D = y2 - y1;
    
    const dot = A * C + B * D;
    const lenSq = C * C + D * D;
    let param = -1;
    
    if (lenSq !== 0) param = dot / lenSq;
    
    let xx, yy;
    
    if (param < 0) {
      xx = x1;
      yy = y1;
    } else if (param > 1) {
      xx = x2;
      yy = y2;
    } else {
      xx = x1 + param * C;
      yy = y1 + param * D;
    }
    
    const dx = px - xx;
    const dy = py - yy;
    return Math.sqrt(dx * dx + dy * dy);
  };

  // Check if a line segment from p1 to p2 passes through any obstacle
  const segmentIntersectsObstacle = useCallback((p1, p2, obstacle) => {
    // Check multiple points along the segment
    const steps = Math.max(
      Math.ceil(Math.abs(p2.x - p1.x)),
      Math.ceil(Math.abs(p2.y - p1.y)),
      10
    );
    
    for (let i = 1; i < steps; i++) {
      const t = i / steps;
      const x = p1.x + (p2.x - p1.x) * t;
      const y = p1.y + (p2.y - p1.y) * t;
      
      if (isPointInObstacle(x, y, obstacle)) {
        return true;
      }
    }
    return false;
  }, [isPointInObstacle]);

  // Get all obstacles including the wall
  const getAllObstacles = useCallback(() => {
    return [...obstacles, wallObstacle];
  }, [obstacles]);

  // Find the point along a segment that's inside an obstacle (for insertion)
  const findSegmentObstacleIntersection = useCallback((p1, p2, obstacle) => {
    const steps = Math.max(
      Math.ceil(Math.abs(p2.x - p1.x)),
      Math.ceil(Math.abs(p2.y - p1.y)),
      20
    );
    
    for (let i = 1; i < steps; i++) {
      const t = i / steps;
      const x = p1.x + (p2.x - p1.x) * t;
      const y = p1.y + (p2.y - p1.y) * t;
      
      if (isPointInObstacle(x, y, obstacle)) {
        return { x, y, t };
      }
    }
    return null;
  }, [isPointInObstacle]);

  const lineIntersectsObstacle = useCallback((p1, p2, obstacle) => {
    const points = [];
    const steps = Math.max(Math.abs(p2.x - p1.x), Math.abs(p2.y - p1.y)) * 2;
    
    for (let i = 0; i <= steps; i++) {
      const t = i / steps;
      const x = p1.x + (p2.x - p1.x) * t;
      const y = p1.y + (p2.y - p1.y) * t;
      
      if (isPointInObstacle(x, y, obstacle)) {
        points.push({ x, y, t });
      }
    }
    
    return points;
  }, [isPointInObstacle]);

  const calculateRepulsionForce = useCallback((point, obstacles) => {
    let forceX = 0;
    let forceY = 0;
    
    for (const obstacle of obstacles) {
      const centerX = (obstacle.x1 + obstacle.x2) / 2;
      const centerY = (obstacle.y1 + obstacle.y2) / 2;
      
      const closestX = Math.max(obstacle.x1, Math.min(point.x, obstacle.x2));
      const closestY = Math.max(obstacle.y1, Math.min(point.y, obstacle.y2));
      
      const dx = point.x - closestX;
      const dy = point.y - closestY;
      const distance = Math.sqrt(dx * dx + dy * dy) || 0.1;
      
      if (distance < REPULSION_RADIUS) {
        const strength = REPULSION_STRENGTH * (1 - distance / REPULSION_RADIUS);
        const normalizedDx = dx / distance || 0;
        const normalizedDy = dy / distance || 0;
        
        forceX += normalizedDx * strength;
        forceY += normalizedDy * strength;
      }
    }
    
    return { x: forceX, y: forceY };
  }, []);

  // Calculate repulsion from a SINGLE specific obstacle
  // Pushes perpendicular to the path direction (left or right of the optimal line)
  // But checks that escape path doesn't go through OTHER obstacles
  const calculateRepulsionFromObstacle = useCallback((point, obstacle, prevPoint, nextPoint) => {
    const allObstacles = getAllObstacles();
    
    // Helper: check if a point is inside ANY obstacle
    const isInsideAnyObstacle = (x, y) => {
      for (const obs of allObstacles) {
        if (isPointInObstacle(x, y, obs)) return true;
      }
      return false;
    };
    
    // Special case: wall obstacle - push away from nearest wall
    if (obstacle.type === 'wall') {
      let forceX = 0;
      let forceY = 0;
      const margin = inflationRadius;
      
      if (point.x < margin) {
        forceX = REPULSION_STRENGTH * 2;
      } else if (point.x > GRID_SIZE - 1 - margin) {
        forceX = -REPULSION_STRENGTH * 2;
      }
      
      if (point.y < margin) {
        forceY = REPULSION_STRENGTH * 2;
      } else if (point.y > GRID_SIZE - 1 - margin) {
        forceY = -REPULSION_STRENGTH * 2;
      }
      
      return { x: forceX, y: forceY };
    }
    
    // Calculate the path direction (from prev to next point)
    let pathDirX, pathDirY;
    
    if (prevPoint && nextPoint) {
      pathDirX = nextPoint.x - prevPoint.x;
      pathDirY = nextPoint.y - prevPoint.y;
    } else {
      pathDirX = goal.x - start.x;
      pathDirY = goal.y - start.y;
    }
    
    // Normalize path direction
    const pathLength = Math.sqrt(pathDirX * pathDirX + pathDirY * pathDirY) || 1;
    pathDirX /= pathLength;
    pathDirY /= pathLength;
    
    // Calculate perpendicular directions (left and right of path)
    const perpLeftX = -pathDirY;
    const perpLeftY = pathDirX;
    const perpRightX = pathDirY;
    const perpRightY = -pathDirX;
    
    // Find how far we need to go in each direction to be clear of ALL obstacles
    let distLeftClear = -1;
    let distRightClear = -1;
    
    // Check left direction
    for (let d = 0.5; d < 25; d += 0.5) {
      const testX = point.x + perpLeftX * d;
      const testY = point.y + perpLeftY * d;
      if (!isInsideAnyObstacle(testX, testY)) {
        distLeftClear = d;
        break;
      }
    }
    
    // Check right direction
    for (let d = 0.5; d < 25; d += 0.5) {
      const testX = point.x + perpRightX * d;
      const testY = point.y + perpRightY * d;
      if (!isInsideAnyObstacle(testX, testY)) {
        distRightClear = d;
        break;
      }
    }
    
    let forceX, forceY;
    
    // Choose the direction that leads to clear space
    if (distLeftClear > 0 && (distRightClear < 0 || distLeftClear <= distRightClear)) {
      // Left is clear (or closer)
      forceX = perpLeftX * REPULSION_STRENGTH * 2;
      forceY = perpLeftY * REPULSION_STRENGTH * 2;
    } else if (distRightClear > 0) {
      // Right is clear
      forceX = perpRightX * REPULSION_STRENGTH * 2;
      forceY = perpRightY * REPULSION_STRENGTH * 2;
    } else {
      // Neither perpendicular direction is clear - push away from obstacle center
      let centerX, centerY;
      
      if (obstacle.type === 'rectangle' || (!obstacle.type && obstacle.x1 !== undefined)) {
        centerX = (obstacle.x1 + obstacle.x2) / 2;
        centerY = (obstacle.y1 + obstacle.y2) / 2;
      } else if (obstacle.type === 'circle') {
        centerX = obstacle.cx;
        centerY = obstacle.cy;
      } else if (obstacle.type === 'triangle') {
        centerX = (obstacle.points[0].x + obstacle.points[1].x + obstacle.points[2].x) / 3;
        centerY = (obstacle.points[0].y + obstacle.points[1].y + obstacle.points[2].y) / 3;
      } else {
        centerX = point.x;
        centerY = point.y;
      }
      
      const dx = point.x - centerX;
      const dy = point.y - centerY;
      const dist = Math.sqrt(dx * dx + dy * dy) || 1;
      
      forceX = (dx / dist) * REPULSION_STRENGTH * 1.5;
      forceY = (dy / dist) * REPULSION_STRENGTH * 1.5;
    }
    
    return { x: forceX, y: forceY };
  }, [isPointInObstacle, start, goal, inflationRadius, getAllObstacles]);

  const initializePath = useCallback(() => {
    const numPoints = 20;
    const points = [];
    
    for (let i = 0; i <= numPoints; i++) {
      const t = i / numPoints;
      points.push({
        x: start.x + (goal.x - start.x) * t,
        y: start.y + (goal.y - start.y) * t,
        isIntersection: false
      });
    }
    
    return points;
  }, [start, goal]);

  const findBoundaryIntersections = useCallback((points) => {
    const boundaryPoints = [];
    
    for (let i = 0; i < points.length - 1; i++) {
      const current = points[i];
      const next = points[i + 1];
      
      // Check if this segment crosses any obstacle boundary
      for (const obstacle of obstacles) {
        const currentInside = isPointInObstacle(current.x, current.y, obstacle);
        const nextInside = isPointInObstacle(next.x, next.y, obstacle);
        
        // Boundary crossing: one point inside, one outside
        if (currentInside !== nextInside) {
          // Find the approximate boundary crossing point
          const crossingPoint = findBoundaryCrossing(current, next, obstacle);
          boundaryPoints.push({
            index: currentInside ? i : i + 1, // Index of the point that's inside
            crossingPoint: crossingPoint,
            obstacle: obstacle,
            isEntry: !currentInside && nextInside, // true if entering obstacle
            isExit: currentInside && !nextInside   // true if exiting obstacle
          });
        }
      }
    }
    
    return boundaryPoints;
  }, [obstacles, isPointInObstacle]);

  const findBoundaryCrossing = useCallback((p1, p2, obstacle) => {
    // Binary search to find the boundary crossing point
    let inside = isPointInObstacle(p1.x, p1.y, obstacle) ? p1 : p2;
    let outside = isPointInObstacle(p1.x, p1.y, obstacle) ? p2 : p1;
    
    for (let i = 0; i < 10; i++) {
      const mid = {
        x: (inside.x + outside.x) / 2,
        y: (inside.y + outside.y) / 2
      };
      
      if (isPointInObstacle(mid.x, mid.y, obstacle)) {
        inside = mid;
      } else {
        outside = mid;
      }
    }
    
    return { x: (inside.x + outside.x) / 2, y: (inside.y + outside.y) / 2 };
  }, [isPointInObstacle]);

  const findIntersections = useCallback((points) => {
    const intersectionIndices = new Set();
    const boundaryData = findBoundaryIntersections(points);
    
    // Only mark the points at boundaries
    boundaryData.forEach(bd => {
      intersectionIndices.add(bd.index);
    });
    
    return Array.from(intersectionIndices);
  }, [findBoundaryIntersections]);

  const stepSimulation = useCallback((currentPoints, targetIndex = null) => {
    // First, find which points are at obstacle boundaries and inside
    const boundaryData = findBoundaryIntersections(currentPoints);
    
    // Find points that are boundary AND inside an obstacle (unresolved)
    const unresolvedBoundaries = boundaryData.filter(bd => 
      isPointInObstacle(currentPoints[bd.index].x, currentPoints[bd.index].y, bd.obstacle)
    );
    
    // If no target specified, pick the first unresolved boundary point
    let activeIndex = targetIndex;
    let activeObstacle = null;
    
    if (activeIndex === null && unresolvedBoundaries.length > 0) {
      activeIndex = unresolvedBoundaries[0].index;
      activeObstacle = unresolvedBoundaries[0].obstacle;
    } else if (activeIndex !== null) {
      // Find the obstacle for the target index
      const targetBoundary = unresolvedBoundaries.find(bd => bd.index === activeIndex);
      if (targetBoundary) {
        activeObstacle = targetBoundary.obstacle;
      }
    }
    
    const newPoints = currentPoints.map((point, index) => {
      if (index === 0 || index === currentPoints.length - 1) {
        return { 
          ...point, 
          isIntersection: false, 
          isBoundary: false, 
          wasPerturbed: false,
          isActive: false 
        };
      }
      
      // Check if this point is a boundary point
      const boundaryInfo = boundaryData.find(bd => bd.index === index);
      const isBoundaryPoint = !!boundaryInfo;
      const isInsideObstacle = boundaryInfo ? 
        isPointInObstacle(point.x, point.y, boundaryInfo.obstacle) : false;
      
      // Only apply forces to the ACTIVE point (the one we're currently resolving)
      const isActivePoint = index === activeIndex;
      
      if (!isActivePoint) {
        // Non-active points: just apply light tension to keep path smooth
        const prev = currentPoints[index - 1];
        const next = currentPoints[index + 1];
        const tensionX = (prev.x + next.x) / 2 - point.x;
        const tensionY = (prev.y + next.y) / 2 - point.y;
        
        return {
          x: Math.max(0, Math.min(GRID_SIZE - 1, point.x + tensionX * 0.05)),
          y: Math.max(0, Math.min(GRID_SIZE - 1, point.y + tensionY * 0.05)),
          force: { x: 0, y: 0 },
          isIntersection: isBoundaryPoint && isInsideObstacle,
          isBoundary: isBoundaryPoint,
          wasPerturbed: false,
          isActive: false
        };
      }
      
      // This is the ACTIVE point - apply repulsion to push it out
      const force = calculateRepulsionForce(point, obstacles);
      
      const prev = currentPoints[index - 1];
      const next = currentPoints[index + 1];
      const tensionX = (prev.x + next.x) / 2 - point.x;
      const tensionY = (prev.y + next.y) / 2 - point.y;
      
      let finalForceX = force.x;
      let finalForceY = force.y;
      let wasPerturbed = false;
      
      // If forces are balanced but still inside, add random perturbation
      const forceMagnitude = Math.sqrt(force.x * force.x + force.y * force.y);
      if (isInsideObstacle && forceMagnitude < FORCE_BALANCE_THRESHOLD) {
        const randomAngle = Math.random() * Math.PI * 2;
        finalForceX += Math.cos(randomAngle) * PERTURBATION_STRENGTH;
        finalForceY += Math.sin(randomAngle) * PERTURBATION_STRENGTH;
        wasPerturbed = true;
      }
      
      const newX = point.x + finalForceX * 0.6 + tensionX * 0.05;
      const newY = point.y + finalForceY * 0.6 + tensionY * 0.05;
      
      return {
        x: Math.max(0, Math.min(GRID_SIZE - 1, newX)),
        y: Math.max(0, Math.min(GRID_SIZE - 1, newY)),
        force: { x: finalForceX, y: finalForceY },
        isIntersection: isInsideObstacle,
        isBoundary: true,
        wasPerturbed: wasPerturbed,
        isActive: true
      };
    });
    
    // Recalculate boundaries for new points
    const newBoundaryData = findBoundaryIntersections(newPoints);
    const newIntersections = [];
    
    newBoundaryData.forEach(bd => {
      if (newPoints[bd.index] && isPointInObstacle(newPoints[bd.index].x, newPoints[bd.index].y, bd.obstacle)) {
        newPoints[bd.index].isIntersection = true;
        newPoints[bd.index].isBoundary = true;
        newIntersections.push(bd.index);
      } else if (newPoints[bd.index]) {
        newPoints[bd.index].isBoundary = true;
      }
    });
    
    // Check if active point is now resolved
    let activeResolved = false;
    if (activeIndex !== null && activeObstacle !== null) {
      activeResolved = !isPointInObstacle(newPoints[activeIndex].x, newPoints[activeIndex].y, activeObstacle);
    }
    
    return { 
      points: newPoints, 
      intersections: newIntersections, 
      boundaryCount: newBoundaryData.length,
      activeIndex: activeIndex,
      activeResolved: activeResolved
    };
  }, [obstacles, calculateRepulsionForce, findBoundaryIntersections, isPointInObstacle]);

  const runFullSimulation = useCallback(() => {
    setAlgorithmPhase('initializing');
    let currentPoints = initializePath().map((p, i, arr) => ({
      ...p,
      isIntersection: false,
      isBoundary: false,
      isActive: false,
      wasPerturbed: false,
      isLocked: i === 0 || i === arr.length - 1
    }));
    
    // Track locked point positions (index -> true)
    const lockedIndices = new Set([0, currentPoints.length - 1]);
    
    const simulationHistory = [{ 
      points: [...currentPoints],
      intersections: [],
      activeIndex: null,
      message: 'Initial straight line path'
    }];
    
    let iterations = 0;
    
    while (iterations < MAX_ITERATIONS) {
      // STEP 1: Find the FIRST segment (from start) that crosses an obstacle
      // We process in order from start to goal
      let firstIntersection = null;
      const allObstacles = getAllObstacles();
      
      for (let i = 0; i < currentPoints.length - 1; i++) {
        const p1 = currentPoints[i];
        const p2 = currentPoints[i + 1];
        
        // Skip if both points are locked and segment is clear
        // (already processed segments between locked points)
        
        for (const obstacle of allObstacles) {
          if (segmentIntersectsObstacle(p1, p2, obstacle)) {
            // Find the ENTRY point (first point where segment enters obstacle)
            const intersection = findSegmentObstacleIntersection(p1, p2, obstacle);
            if (intersection) {
              firstIntersection = {
                segmentIndex: i,
                intersectionPoint: intersection,
                obstacle: obstacle
              };
              break;
            }
          }
        }
        if (firstIntersection) break;
      }
      
      // STEP 2: If no intersection found, we're done!
      if (!firstIntersection) {
        simulationHistory.push({
          points: currentPoints.map(p => ({ ...p, isActive: false })),
          intersections: [],
          activeIndex: null,
          message: '✓ Path completely clear of all obstacles!'
        });
        console.log(`Solved in ${iterations} iterations!`);
        break;
      }
      
      // STEP 3: Insert a new point at the intersection
      const insertIndex = firstIntersection.segmentIndex + 1;
      const newPoint = {
        x: firstIntersection.intersectionPoint.x,
        y: firstIntersection.intersectionPoint.y,
        isIntersection: true,
        isBoundary: true,
        isActive: true,
        wasPerturbed: false,
        isLocked: false
      };
      
      // Insert point
      currentPoints = [
        ...currentPoints.slice(0, insertIndex),
        newPoint,
        ...currentPoints.slice(insertIndex)
      ];
      
      // Update locked indices (shift all indices >= insertIndex)
      const newLockedIndices = new Set();
      lockedIndices.forEach(idx => {
        if (idx >= insertIndex) {
          newLockedIndices.add(idx + 1);
        } else {
          newLockedIndices.add(idx);
        }
      });
      lockedIndices.clear();
      newLockedIndices.forEach(idx => lockedIndices.add(idx));
      
      // Mark points
      currentPoints.forEach((p, i) => {
        p.isLocked = lockedIndices.has(i);
        p.isActive = i === insertIndex;
        p.isIntersection = i === insertIndex;
      });
      
      const activeObstacle = firstIntersection.obstacle;
      let activeIndex = insertIndex;
      
      // Get the LOCKED points before and after for path direction
      // Start point (index 0) is always locked
      // We use start and the next locked point (or goal) for direction
      let prevLockedIdx = 0; // Start is always locked
      let nextLockedIdx = currentPoints.length - 1; // Goal is always locked
      
      // Find closest locked point after activeIndex
      for (let i = activeIndex + 1; i < currentPoints.length; i++) {
        if (lockedIndices.has(i)) {
          nextLockedIdx = i;
          break;
        }
      }
      
      // Find closest locked point before activeIndex
      for (let i = activeIndex - 1; i >= 0; i--) {
        if (lockedIndices.has(i)) {
          prevLockedIdx = i;
          break;
        }
      }
      
      const pathRefPrev = currentPoints[prevLockedIdx];
      const pathRefNext = currentPoints[nextLockedIdx];
      
      simulationHistory.push({
        points: currentPoints.map(p => ({ ...p })),
        intersections: [activeIndex],
        activeIndex: activeIndex,
        message: `Inserted point at segment ${firstIntersection.segmentIndex}. Pushing out...`
      });
      iterations++;
      
      // STEP 4: Push this point out until it's completely outside THIS obstacle
      let pushIterations = 0;
      
      while (pushIterations < 300 && 
             isPointInObstacle(currentPoints[activeIndex].x, currentPoints[activeIndex].y, activeObstacle)) {
        
        const point = currentPoints[activeIndex];
        
        // Calculate repulsion perpendicular to the path direction
        const force = calculateRepulsionFromObstacle(point, activeObstacle, pathRefPrev, pathRefNext);
        
        let finalForceX = force.x;
        let finalForceY = force.y;
        let wasPerturbed = false;
        
        const forceMagnitude = Math.sqrt(force.x * force.x + force.y * force.y);
        if (forceMagnitude < FORCE_BALANCE_THRESHOLD) {
          const randomAngle = Math.random() * Math.PI * 2;
          finalForceX += Math.cos(randomAngle) * PERTURBATION_STRENGTH;
          finalForceY += Math.sin(randomAngle) * PERTURBATION_STRENGTH;
          wasPerturbed = true;
        }
        
        // Only update the active point
        currentPoints[activeIndex] = {
          ...currentPoints[activeIndex],
          x: Math.max(0, Math.min(GRID_SIZE - 1, point.x + finalForceX * stepSize)),
          y: Math.max(0, Math.min(GRID_SIZE - 1, point.y + finalForceY * stepSize)),
          force: { x: finalForceX, y: finalForceY },
          isActive: true,
          wasPerturbed: wasPerturbed
        };
        
        const stillInside = isPointInObstacle(
          currentPoints[activeIndex].x, 
          currentPoints[activeIndex].y, 
          activeObstacle
        );
        
        simulationHistory.push({
          points: currentPoints.map(p => ({ ...p })),
          intersections: stillInside ? [activeIndex] : [],
          activeIndex: activeIndex,
          message: stillInside 
            ? `Pushing point ${activeIndex}... (${lockedIndices.size - 2} locked)`
            : `Point ${activeIndex} is now outside! Locking.`
        });
        
        pushIterations++;
        iterations++;
        if (iterations >= MAX_ITERATIONS) break;
      }
      
      // STEP 5: Lock the point now that it's outside
      lockedIndices.add(activeIndex);
      currentPoints[activeIndex].isLocked = true;
      currentPoints[activeIndex].isActive = false;
      currentPoints[activeIndex].isIntersection = false;
      
      // STEP 6: Remove any unlocked intermediate points between locked points
      // This cleans up points that were created but are no longer needed
      const cleanedPoints = [];
      for (let i = 0; i < currentPoints.length; i++) {
        if (lockedIndices.has(i) || i === 0 || i === currentPoints.length - 1) {
          cleanedPoints.push(currentPoints[i]);
        }
      }
      
      // Rebuild locked indices for cleaned points
      lockedIndices.clear();
      cleanedPoints.forEach((p, i) => {
        if (p.isLocked) {
          lockedIndices.add(i);
        }
      });
      lockedIndices.add(0);
      lockedIndices.add(cleanedPoints.length - 1);
      
      currentPoints = cleanedPoints;
      currentPoints.forEach((p, i) => {
        p.isLocked = lockedIndices.has(i);
      });
      
      if (iterations >= MAX_ITERATIONS) break;
    }
    
    if (iterations >= MAX_ITERATIONS) {
      console.log(`Reached max iterations (${MAX_ITERATIONS})`);
    }
    
    setHistory(simulationHistory);
    setCurrentStep(0);
    setPathPoints(simulationHistory[0].points);
    setIntersections(simulationHistory[0].intersections);
    setAlgorithmPhase('ready');
  }, [initializePath, segmentIntersectsObstacle, findSegmentObstacleIntersection, isPointInObstacle, calculateRepulsionFromObstacle, getAllObstacles, stepSize]);

  const animate = useCallback(() => {
    if (history.length === 0) return;
    
    // Clear any existing animation
    if (animationIntervalRef.current) {
      clearInterval(animationIntervalRef.current);
    }
    
    setIsAnimating(true);
    setAlgorithmPhase('animating');
    let step = 0;
    
    animationIntervalRef.current = setInterval(() => {
      if (step >= history.length) {
        clearInterval(animationIntervalRef.current);
        animationIntervalRef.current = null;
        setIsAnimating(false);
        setAlgorithmPhase('complete');
        setStepMessage('');
        return;
      }
      
      setCurrentStep(step);
      setPathPoints(history[step].points);
      setIntersections(history[step].intersections);
      setStepMessage(history[step].message || '');
      
      const forces = history[step].points
        .filter(p => p.force && (Math.abs(p.force.x) > 0.01 || Math.abs(p.force.y) > 0.01))
        .map(p => ({ x: p.x, y: p.y, fx: p.force.x, fy: p.force.y }));
      setForceVectors(forces);
      
      step++;
    }, 150);
  }, [history]);
  
  // Stop animation function
  const stopAnimation = useCallback(() => {
    if (animationIntervalRef.current) {
      clearInterval(animationIntervalRef.current);
      animationIntervalRef.current = null;
    }
    setIsAnimating(false);
    setAlgorithmPhase('ready');
  }, []);

  const handleGridClick = (e) => {
    const rect = e.currentTarget.getBoundingClientRect();
    const x = Math.floor((e.clientX - rect.left) / CELL_SIZE);
    const y = Math.floor((e.clientY - rect.top) / CELL_SIZE);
    
    if (editMode === 'start') {
      setStart({ x, y });
      setEditMode(null);
    } else if (editMode === 'goal') {
      setGoal({ x, y });
      setEditMode(null);
    } else if (editMode === 'obstacle') {
      if (selectedShape === 'rectangle') {
        if (!tempObstacle) {
          setTempObstacle({ type: 'rectangle', x1: x, y1: y, x2: x, y2: y });
        } else {
          const newObstacle = {
            type: 'rectangle',
            x1: Math.min(tempObstacle.x1, x),
            y1: Math.min(tempObstacle.y1, y),
            x2: Math.max(tempObstacle.x1, x),
            y2: Math.max(tempObstacle.y1, y)
          };
          setObstacles([...obstacles, newObstacle]);
          setTempObstacle(null);
          // Don't reset editMode - allow continuous adding
        }
      } else if (selectedShape === 'circle') {
        if (!tempObstacle) {
          setTempObstacle({ type: 'circle', cx: x, cy: y, radius: 0 });
        } else {
          const dx = x - tempObstacle.cx;
          const dy = y - tempObstacle.cy;
          const radius = Math.max(1, Math.sqrt(dx * dx + dy * dy));
          const newObstacle = {
            type: 'circle',
            cx: tempObstacle.cx,
            cy: tempObstacle.cy,
            radius: radius
          };
          setObstacles([...obstacles, newObstacle]);
          setTempObstacle(null);
          // Don't reset editMode - allow continuous adding
        }
      } else if (selectedShape === 'triangle') {
        const newPoints = [...trianglePoints, { x, y }];
        setTrianglePoints(newPoints);
        
        if (newPoints.length === 3) {
          const newObstacle = {
            type: 'triangle',
            points: newPoints
          };
          setObstacles([...obstacles, newObstacle]);
          setTrianglePoints([]);
          // Don't reset editMode - allow continuous adding
        }
      }
    }
  };

  const handleMouseMove = (e) => {
    if (editMode === 'obstacle' && tempObstacle) {
      const rect = e.currentTarget.getBoundingClientRect();
      const x = Math.floor((e.clientX - rect.left) / CELL_SIZE);
      const y = Math.floor((e.clientY - rect.top) / CELL_SIZE);
      
      if (selectedShape === 'rectangle') {
        setTempObstacle({ ...tempObstacle, x2: x, y2: y });
      } else if (selectedShape === 'circle') {
        const dx = x - tempObstacle.cx;
        const dy = y - tempObstacle.cy;
        const radius = Math.sqrt(dx * dx + dy * dy);
        setTempObstacle({ ...tempObstacle, radius: radius });
      }
    }
  };

  const clearObstacles = () => {
    setObstacles([]);
    setHistory([]);
    setPathPoints([]);
    setCurrentStep(0);
    setTrianglePoints([]);
    setTempObstacle(null);
  };

  const reset = () => {
    // Stop any running animation
    if (animationIntervalRef.current) {
      clearInterval(animationIntervalRef.current);
      animationIntervalRef.current = null;
    }
    setIsAnimating(false);
    setHistory([]);
    setPathPoints([]);
    setCurrentStep(0);
    setIntersections([]);
    setForceVectors([]);
    setAlgorithmPhase('idle');
    setStepMessage('');
  };

  useEffect(() => {
    reset();
  }, [start, goal, obstacles]);

  const getPhaseDescription = () => {
    switch (algorithmPhase) {
      case 'idle': return 'Click "Initialize & Run" to start the simulation';
      case 'initializing': return 'Drawing initial straight line from S to G...';
      case 'ready': 
        const finalIntersections = history.length > 0 ? history[history.length - 1].intersections.length : 0;
        if (finalIntersections === 0) {
          return `✓ SOLVED in ${history.length} steps! Click "Animate" to visualize.`;
        } else {
          return `Stopped at ${history.length} steps with ${finalIntersections} intersections. Click "Animate" to visualize.`;
        }
      case 'animating': return `Step ${currentStep + 1}/${history.length}: Repelling path from obstacles...`;
      case 'complete': 
        const endIntersections = history.length > 0 ? history[history.length - 1].intersections.length : 0;
        return endIntersections === 0 
          ? '✓ Path planning complete! Final path avoids all obstacles.'
          : `Animation complete. ${endIntersections} intersections remain.`;
      default: return '';
    }
  };

  return (
    <div style={{
      minHeight: '100vh',
      background: 'linear-gradient(135deg, #0a0a0f 0%, #1a1a2e 50%, #0f0f1a 100%)',
      padding: '24px',
      fontFamily: '"JetBrains Mono", "Fira Code", monospace'
    }}>
      <style>{`
        @import url('https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;500;600;700&family=Space+Grotesk:wght@400;500;600;700&display=swap');
        
        @keyframes pulse {
          0%, 100% { opacity: 1; }
          50% { opacity: 0.5; }
        }
        
        @keyframes glow {
          0%, 100% { filter: drop-shadow(0 0 8px currentColor); }
          50% { filter: drop-shadow(0 0 16px currentColor); }
        }
        
        @keyframes scan {
          0% { transform: translateY(-100%); }
          100% { transform: translateY(100%); }
        }
        
        .btn {
          padding: 10px 20px;
          border: 1px solid;
          border-radius: 4px;
          font-family: inherit;
          font-size: 13px;
          font-weight: 500;
          cursor: pointer;
          transition: all 0.2s ease;
          text-transform: uppercase;
          letter-spacing: 1px;
        }
        
        .btn:disabled {
          opacity: 0.4;
          cursor: not-allowed;
        }
        
        .btn-primary {
          background: linear-gradient(135deg, #00ff88 0%, #00cc6a 100%);
          border-color: #00ff88;
          color: #0a0a0f;
        }
        
        .btn-primary:hover:not(:disabled) {
          box-shadow: 0 0 20px rgba(0, 255, 136, 0.4);
          transform: translateY(-1px);
        }
        
        .btn-secondary {
          background: transparent;
          border-color: #4a9eff;
          color: #4a9eff;
        }
        
        .btn-secondary:hover:not(:disabled) {
          background: rgba(74, 158, 255, 0.1);
          box-shadow: 0 0 20px rgba(74, 158, 255, 0.3);
        }
        
        .btn-danger {
          background: transparent;
          border-color: #ff4a6a;
          color: #ff4a6a;
        }
        
        .btn-danger:hover:not(:disabled) {
          background: rgba(255, 74, 106, 0.1);
          box-shadow: 0 0 20px rgba(255, 74, 106, 0.3);
        }
        
        .btn-active {
          background: rgba(255, 200, 50, 0.2) !important;
          border-color: #ffc832 !important;
          color: #ffc832 !important;
          box-shadow: 0 0 15px rgba(255, 200, 50, 0.3);
        }
      `}</style>
      
      <div style={{ maxWidth: '1200px', margin: '0 auto' }}>
        {/* Header */}
        <div style={{ marginBottom: '32px', textAlign: 'center' }}>
          <h1 style={{
            fontSize: '28px',
            fontWeight: 700,
            color: '#00ff88',
            margin: 0,
            fontFamily: '"Space Grotesk", sans-serif',
            textShadow: '0 0 30px rgba(0, 255, 136, 0.5)',
            letterSpacing: '2px'
          }}>
            ⚡ LAZY COULOMB PLANNER
          </h1>
          <p style={{
            color: '#6a7a8a',
            fontSize: '13px',
            marginTop: '8px',
            letterSpacing: '1px'
          }}>
            Path Planning via Electrostatic Repulsion
          </p>
        </div>

        <div style={{ display: 'flex', gap: '24px', flexWrap: 'wrap' }}>
          {/* Grid Container */}
          <div style={{
            background: 'rgba(20, 20, 35, 0.8)',
            borderRadius: '8px',
            padding: '20px',
            border: '1px solid rgba(100, 100, 140, 0.3)',
            boxShadow: '0 10px 40px rgba(0, 0, 0, 0.5), inset 0 1px 0 rgba(255, 255, 255, 0.05)'
          }}>
            {/* Status Bar */}
            <div style={{
              background: 'rgba(0, 0, 0, 0.4)',
              borderRadius: '4px',
              padding: '12px 16px',
              marginBottom: '16px',
              border: '1px solid rgba(100, 100, 140, 0.2)',
              display: 'flex',
              alignItems: 'center',
              gap: '12px'
            }}>
              <div style={{
                width: '8px',
                height: '8px',
                borderRadius: '50%',
                background: algorithmPhase === 'animating' ? '#00ff00' : 
                           algorithmPhase === 'complete' ? '#00ff88' : '#4a9eff',
                animation: algorithmPhase === 'animating' ? 'pulse 1s infinite' : 'none',
                boxShadow: `0 0 10px ${algorithmPhase === 'animating' ? '#00ff00' : 
                           algorithmPhase === 'complete' ? '#00ff88' : '#4a9eff'}`
              }} />
              <span style={{ color: '#a0a8b0', fontSize: '12px', flex: 1 }}>
                {algorithmPhase === 'animating' && stepMessage ? stepMessage : getPhaseDescription()}
              </span>
              {history.length > 0 && (
                <span style={{ color: '#4a9eff', fontSize: '12px' }}>
                  Step: {currentStep + 1}/{history.length}
                </span>
              )}
            </div>

            {/* Grid */}
            <svg
              width={GRID_SIZE * CELL_SIZE}
              height={GRID_SIZE * CELL_SIZE}
              onClick={handleGridClick}
              onMouseMove={handleMouseMove}
              style={{
                background: 'linear-gradient(180deg, #0a0a12 0%, #12121a 100%)',
                borderRadius: '4px',
                cursor: editMode ? 'crosshair' : 'default',
                border: '1px solid rgba(100, 100, 140, 0.3)'
              }}
            >
              {/* Grid lines */}
              <defs>
                <pattern id="grid" width={CELL_SIZE} height={CELL_SIZE} patternUnits="userSpaceOnUse">
                  <path d={`M ${CELL_SIZE} 0 L 0 0 0 ${CELL_SIZE}`} fill="none" stroke="rgba(60, 70, 90, 0.3)" strokeWidth="0.5"/>
                </pattern>
                <linearGradient id="pathGradient" x1="0%" y1="0%" x2="100%" y2="100%">
                  <stop offset="0%" stopColor="#00ff88" />
                  <stop offset="100%" stopColor="#4a9eff" />
                </linearGradient>
                <filter id="glow">
                  <feGaussianBlur stdDeviation="2" result="coloredBlur"/>
                  <feMerge>
                    <feMergeNode in="coloredBlur"/>
                    <feMergeNode in="SourceGraphic"/>
                  </feMerge>
                </filter>
              </defs>
              <rect width="100%" height="100%" fill="url(#grid)" />
              
              {/* Initial straight line (faded) */}
              {pathPoints.length > 0 && (
                <line
                  x1={start.x * CELL_SIZE + CELL_SIZE / 2}
                  y1={start.y * CELL_SIZE + CELL_SIZE / 2}
                  x2={goal.x * CELL_SIZE + CELL_SIZE / 2}
                  y2={goal.y * CELL_SIZE + CELL_SIZE / 2}
                  stroke="rgba(255, 200, 50, 0.15)"
                  strokeWidth="2"
                  strokeDasharray="8 4"
                />
              )}

              {/* Wall boundary with inflation zone */}
              <rect
                x={inflationRadius * CELL_SIZE}
                y={inflationRadius * CELL_SIZE}
                width={(GRID_SIZE - 2 * inflationRadius) * CELL_SIZE}
                height={(GRID_SIZE - 2 * inflationRadius) * CELL_SIZE}
                fill="none"
                stroke="#ff4a6a"
                strokeWidth="1"
                strokeDasharray="4 3"
                opacity="0.5"
              />
              {/* Wall boundary solid */}
              <rect
                x="0"
                y="0"
                width={GRID_SIZE * CELL_SIZE}
                height={GRID_SIZE * CELL_SIZE}
                fill="none"
                stroke="#ff4a6a"
                strokeWidth="2"
              />

              {/* Obstacles with charge visualization */}
              {obstacles.map((obs, i) => (
                <g key={i}>
                  {obs.type === 'rectangle' || (!obs.type && obs.x1 !== undefined) ? (
                    <>
                      {/* Inflation zone (dotted) */}
                      <rect
                        x={(obs.x1 - inflationRadius) * CELL_SIZE}
                        y={(obs.y1 - inflationRadius) * CELL_SIZE}
                        width={(obs.x2 - obs.x1 + inflationRadius * 2) * CELL_SIZE}
                        height={(obs.y2 - obs.y1 + inflationRadius * 2) * CELL_SIZE}
                        fill="none"
                        stroke="#ff4a6a"
                        strokeWidth="1"
                        strokeDasharray="4 3"
                        opacity="0.5"
                        rx="4"
                      />
                      {/* Obstacle body */}
                      <rect
                        x={obs.x1 * CELL_SIZE}
                        y={obs.y1 * CELL_SIZE}
                        width={(obs.x2 - obs.x1) * CELL_SIZE}
                        height={(obs.y2 - obs.y1) * CELL_SIZE}
                        fill="rgba(255, 74, 106, 0.3)"
                        stroke="#ff4a6a"
                        strokeWidth="2"
                        rx="2"
                      />
                      {/* Charge symbol */}
                      <text
                        x={(obs.x1 + obs.x2) / 2 * CELL_SIZE}
                        y={(obs.y1 + obs.y2) / 2 * CELL_SIZE + 5}
                        textAnchor="middle"
                        fill="#ff4a6a"
                        fontSize="16"
                        fontWeight="bold"
                      >−</text>
                    </>
                  ) : obs.type === 'circle' ? (
                    <>
                      {/* Inflation zone (dotted) */}
                      <circle
                        cx={obs.cx * CELL_SIZE + CELL_SIZE / 2}
                        cy={obs.cy * CELL_SIZE + CELL_SIZE / 2}
                        r={(obs.radius + inflationRadius) * CELL_SIZE}
                        fill="none"
                        stroke="#ff4a6a"
                        strokeWidth="1"
                        strokeDasharray="4 3"
                        opacity="0.5"
                      />
                      {/* Obstacle body */}
                      <circle
                        cx={obs.cx * CELL_SIZE + CELL_SIZE / 2}
                        cy={obs.cy * CELL_SIZE + CELL_SIZE / 2}
                        r={obs.radius * CELL_SIZE}
                        fill="rgba(255, 74, 106, 0.3)"
                        stroke="#ff4a6a"
                        strokeWidth="2"
                      />
                      {/* Charge symbol */}
                      <text
                        x={obs.cx * CELL_SIZE + CELL_SIZE / 2}
                        y={obs.cy * CELL_SIZE + CELL_SIZE / 2 + 5}
                        textAnchor="middle"
                        fill="#ff4a6a"
                        fontSize="16"
                        fontWeight="bold"
                      >−</text>
                    </>
                  ) : obs.type === 'triangle' ? (
                    <>
                      {/* Inflation zone (dotted) - approximate with scaled triangle */}
                      {(() => {
                        const cx = (obs.points[0].x + obs.points[1].x + obs.points[2].x) / 3;
                        const cy = (obs.points[0].y + obs.points[1].y + obs.points[2].y) / 3;
                        const inflatedPoints = obs.points.map(p => {
                          const dx = p.x - cx;
                          const dy = p.y - cy;
                          const dist = Math.sqrt(dx * dx + dy * dy) || 1;
                          const scale = (dist + inflationRadius) / dist;
                          return {
                            x: cx + dx * scale,
                            y: cy + dy * scale
                          };
                        });
                        return (
                          <polygon
                            points={inflatedPoints.map(p => 
                              `${p.x * CELL_SIZE + CELL_SIZE / 2},${p.y * CELL_SIZE + CELL_SIZE / 2}`
                            ).join(' ')}
                            fill="none"
                            stroke="#ff4a6a"
                            strokeWidth="1"
                            strokeDasharray="4 3"
                            opacity="0.5"
                          />
                        );
                      })()}
                      {/* Obstacle body */}
                      <polygon
                        points={obs.points.map(p => 
                          `${p.x * CELL_SIZE + CELL_SIZE / 2},${p.y * CELL_SIZE + CELL_SIZE / 2}`
                        ).join(' ')}
                        fill="rgba(255, 74, 106, 0.3)"
                        stroke="#ff4a6a"
                        strokeWidth="2"
                      />
                      {/* Charge symbol */}
                      <text
                        x={(obs.points[0].x + obs.points[1].x + obs.points[2].x) / 3 * CELL_SIZE + CELL_SIZE / 2}
                        y={(obs.points[0].y + obs.points[1].y + obs.points[2].y) / 3 * CELL_SIZE + CELL_SIZE / 2 + 5}
                        textAnchor="middle"
                        fill="#ff4a6a"
                        fontSize="16"
                        fontWeight="bold"
                      >−</text>
                    </>
                  ) : null}
                </g>
              ))}
              
              <defs>
                <radialGradient id="obstacleGradient">
                  <stop offset="0%" stopColor="#ff4a6a" stopOpacity="0.4" />
                  <stop offset="100%" stopColor="#ff4a6a" stopOpacity="0" />
                </radialGradient>
              </defs>

              {/* Temp obstacle while drawing */}
              {tempObstacle && tempObstacle.type === 'rectangle' && (
                <rect
                  x={Math.min(tempObstacle.x1, tempObstacle.x2) * CELL_SIZE}
                  y={Math.min(tempObstacle.y1, tempObstacle.y2) * CELL_SIZE}
                  width={Math.abs(tempObstacle.x2 - tempObstacle.x1) * CELL_SIZE}
                  height={Math.abs(tempObstacle.y2 - tempObstacle.y1) * CELL_SIZE}
                  fill="rgba(255, 200, 50, 0.2)"
                  stroke="#ffc832"
                  strokeWidth="2"
                  strokeDasharray="4 2"
                />
              )}
              {tempObstacle && tempObstacle.type === 'circle' && (
                <circle
                  cx={tempObstacle.cx * CELL_SIZE + CELL_SIZE / 2}
                  cy={tempObstacle.cy * CELL_SIZE + CELL_SIZE / 2}
                  r={tempObstacle.radius * CELL_SIZE}
                  fill="rgba(255, 200, 50, 0.2)"
                  stroke="#ffc832"
                  strokeWidth="2"
                  strokeDasharray="4 2"
                />
              )}
              {/* Triangle points being placed */}
              {trianglePoints.length > 0 && (
                <>
                  {trianglePoints.map((p, i) => (
                    <circle
                      key={i}
                      cx={p.x * CELL_SIZE + CELL_SIZE / 2}
                      cy={p.y * CELL_SIZE + CELL_SIZE / 2}
                      r="5"
                      fill="#ffc832"
                    />
                  ))}
                  {trianglePoints.length >= 2 && (
                    <polyline
                      points={trianglePoints.map(p => 
                        `${p.x * CELL_SIZE + CELL_SIZE / 2},${p.y * CELL_SIZE + CELL_SIZE / 2}`
                      ).join(' ')}
                      fill="none"
                      stroke="#ffc832"
                      strokeWidth="2"
                      strokeDasharray="4 2"
                    />
                  )}
                </>
              )}

              {/* Force vectors */}
              {forceVectors.map((f, i) => (
                <g key={i}>
                  <line
                    x1={f.x * CELL_SIZE + CELL_SIZE / 2}
                    y1={f.y * CELL_SIZE + CELL_SIZE / 2}
                    x2={(f.x + f.fx * arrowScale) * CELL_SIZE + CELL_SIZE / 2}
                    y2={(f.y + f.fy * arrowScale) * CELL_SIZE + CELL_SIZE / 2}
                    stroke="#ffc832"
                    strokeWidth="2"
                    opacity="0.8"
                    markerEnd="url(#arrowhead)"
                  />
                </g>
              ))}
              
              <defs>
                <marker id="arrowhead" markerWidth="10" markerHeight="7" refX="9" refY="3.5" orient="auto">
                  <polygon points="0 0, 10 3.5, 0 7" fill="#ffc832" />
                </marker>
              </defs>

              {/* Path */}
              {pathPoints.length > 1 && (
                <g filter="url(#glow)">
                  {(() => {
                    const displayPoints = getDisplayPoints(pathPoints);
                    return (
                      <polyline
                        points={displayPoints.map(p => 
                          `${p.x * CELL_SIZE + CELL_SIZE / 2},${p.y * CELL_SIZE + CELL_SIZE / 2}`
                        ).join(' ')}
                        fill="none"
                        stroke="url(#pathGradient)"
                        strokeWidth="3"
                        strokeLinecap="round"
                        strokeLinejoin="round"
                      />
                    );
                  })()}
                </g>
              )}

              {/* Path points */}
              {pathPoints.map((point, i) => (
                <g key={i}>
                  {/* Active point indicator - large pulsing ring */}
                  {point.isActive && (
                    <circle
                      cx={point.x * CELL_SIZE + CELL_SIZE / 2}
                      cy={point.y * CELL_SIZE + CELL_SIZE / 2}
                      r="14"
                      fill="none"
                      stroke="#00ff00"
                      strokeWidth="3"
                      opacity="0.8"
                      style={{ animation: 'pulse 0.8s infinite' }}
                    />
                  )}
                  {/* Locked point indicator - square border */}
                  {point.isLocked && i !== 0 && i !== pathPoints.length - 1 && (
                    <rect
                      x={point.x * CELL_SIZE + CELL_SIZE / 2 - 8}
                      y={point.y * CELL_SIZE + CELL_SIZE / 2 - 8}
                      width="16"
                      height="16"
                      fill="none"
                      stroke="#00ffff"
                      strokeWidth="2"
                      rx="2"
                      style={{ filter: 'drop-shadow(0 0 4px #00ffff)' }}
                    />
                  )}
                  {/* Perturbation indicator */}
                  {point.wasPerturbed && (
                    <circle
                      cx={point.x * CELL_SIZE + CELL_SIZE / 2}
                      cy={point.y * CELL_SIZE + CELL_SIZE / 2}
                      r="10"
                      fill="none"
                      stroke="#ff00ff"
                      strokeWidth="2"
                      opacity="0.7"
                      style={{ animation: 'pulse 0.5s ease-out' }}
                    />
                  )}
                  <circle
                    cx={point.x * CELL_SIZE + CELL_SIZE / 2}
                    cy={point.y * CELL_SIZE + CELL_SIZE / 2}
                    r={point.isActive ? 7 : point.isLocked ? 5 : point.isIntersection ? 6 : 3}
                    fill={
                      point.isActive ? '#00ff00' :
                      point.isLocked ? '#00ffff' :
                      point.wasPerturbed ? '#ff00ff' : 
                      point.isIntersection ? '#ffc832' : 
                      '#4a9eff'
                    }
                    stroke={point.isActive ? '#00ff00' : point.isLocked ? '#00ffff' : point.isIntersection ? '#ffc832' : 'none'}
                    strokeWidth="2"
                    style={{
                      filter: point.isActive 
                        ? 'drop-shadow(0 0 10px #00ff00)' 
                        : point.isLocked
                          ? 'drop-shadow(0 0 6px #00ffff)'
                          : point.isIntersection 
                            ? 'drop-shadow(0 0 6px #ffc832)' 
                            : 'none'
                    }}
                  />
                </g>
              ))}

              {/* Start point */}
              <g>
                <circle
                  cx={start.x * CELL_SIZE + CELL_SIZE / 2}
                  cy={start.y * CELL_SIZE + CELL_SIZE / 2}
                  r="12"
                  fill="rgba(0, 255, 136, 0.2)"
                  stroke="#00ff88"
                  strokeWidth="2"
                />
                <text
                  x={start.x * CELL_SIZE + CELL_SIZE / 2}
                  y={start.y * CELL_SIZE + CELL_SIZE / 2 + 5}
                  textAnchor="middle"
                  fill="#00ff88"
                  fontSize="14"
                  fontWeight="bold"
                >S</text>
              </g>

              {/* Goal point */}
              <g>
                <circle
                  cx={goal.x * CELL_SIZE + CELL_SIZE / 2}
                  cy={goal.y * CELL_SIZE + CELL_SIZE / 2}
                  r="12"
                  fill="rgba(74, 158, 255, 0.2)"
                  stroke="#4a9eff"
                  strokeWidth="2"
                />
                <text
                  x={goal.x * CELL_SIZE + CELL_SIZE / 2}
                  y={goal.y * CELL_SIZE + CELL_SIZE / 2 + 5}
                  textAnchor="middle"
                  fill="#4a9eff"
                  fontSize="14"
                  fontWeight="bold"
                >G</text>
              </g>
            </svg>
          </div>

          {/* Control Panel */}
          <div style={{
            flex: 1,
            minWidth: '280px',
            display: 'flex',
            flexDirection: 'column',
            gap: '16px'
          }}>
            {/* Algorithm Controls */}
            <div style={{
              background: 'rgba(20, 20, 35, 0.8)',
              borderRadius: '8px',
              padding: '20px',
              border: '1px solid rgba(100, 100, 140, 0.3)'
            }}>
              <h3 style={{
                color: '#00ff88',
                fontSize: '12px',
                fontWeight: 600,
                marginBottom: '16px',
                letterSpacing: '2px',
                textTransform: 'uppercase'
              }}>⚡ Algorithm Controls</h3>
              
              <div style={{ display: 'flex', flexDirection: 'column', gap: '10px' }}>
                <button
                  className="btn btn-primary"
                  onClick={runFullSimulation}
                  disabled={isAnimating}
                >
                  Initialize & Run
                </button>
                <button
                  className="btn btn-secondary"
                  onClick={isAnimating ? stopAnimation : animate}
                  disabled={!isAnimating && history.length === 0}
                >
                  {isAnimating ? 'Stop' : 'Animate Steps'}
                </button>
                <button
                  className="btn btn-danger"
                  onClick={reset}
                >
                  Reset
                </button>
              </div>
            </div>

            {/* Edit Tools */}
            <div style={{
              background: 'rgba(20, 20, 35, 0.8)',
              borderRadius: '8px',
              padding: '20px',
              border: '1px solid rgba(100, 100, 140, 0.3)'
            }}>
              <h3 style={{
                color: '#4a9eff',
                fontSize: '12px',
                fontWeight: 600,
                marginBottom: '16px',
                letterSpacing: '2px',
                textTransform: 'uppercase'
              }}>🔧 Edit Tools</h3>
              
              <div style={{ display: 'flex', flexDirection: 'column', gap: '10px' }}>
                <button
                  className={`btn btn-secondary ${editMode === 'start' ? 'btn-active' : ''}`}
                  onClick={() => setEditMode(editMode === 'start' ? null : 'start')}
                >
                  Set Start (S)
                </button>
                <button
                  className={`btn btn-secondary ${editMode === 'goal' ? 'btn-active' : ''}`}
                  onClick={() => setEditMode(editMode === 'goal' ? null : 'goal')}
                >
                  Set Goal (G)
                </button>
                
                {/* Shape selector */}
                <div style={{ 
                  display: 'flex', 
                  gap: '6px',
                  marginTop: '8px'
                }}>
                  <button
                    style={{
                      flex: 1,
                      padding: '8px',
                      background: selectedShape === 'rectangle' ? 'rgba(255, 74, 106, 0.3)' : 'transparent',
                      border: `1px solid ${selectedShape === 'rectangle' ? '#ff4a6a' : 'rgba(100, 100, 140, 0.5)'}`,
                      borderRadius: '4px',
                      color: selectedShape === 'rectangle' ? '#ff4a6a' : '#6a7a8a',
                      cursor: 'pointer',
                      fontSize: '18px'
                    }}
                    onClick={() => setSelectedShape('rectangle')}
                    title="Rectangle"
                  >▢</button>
                  <button
                    style={{
                      flex: 1,
                      padding: '8px',
                      background: selectedShape === 'circle' ? 'rgba(255, 74, 106, 0.3)' : 'transparent',
                      border: `1px solid ${selectedShape === 'circle' ? '#ff4a6a' : 'rgba(100, 100, 140, 0.5)'}`,
                      borderRadius: '4px',
                      color: selectedShape === 'circle' ? '#ff4a6a' : '#6a7a8a',
                      cursor: 'pointer',
                      fontSize: '18px'
                    }}
                    onClick={() => setSelectedShape('circle')}
                    title="Circle"
                  >○</button>
                  <button
                    style={{
                      flex: 1,
                      padding: '8px',
                      background: selectedShape === 'triangle' ? 'rgba(255, 74, 106, 0.3)' : 'transparent',
                      border: `1px solid ${selectedShape === 'triangle' ? '#ff4a6a' : 'rgba(100, 100, 140, 0.5)'}`,
                      borderRadius: '4px',
                      color: selectedShape === 'triangle' ? '#ff4a6a' : '#6a7a8a',
                      cursor: 'pointer',
                      fontSize: '18px'
                    }}
                    onClick={() => setSelectedShape('triangle')}
                    title="Triangle"
                  >△</button>
                </div>
                
                <button
                  className={`btn btn-secondary ${editMode === 'obstacle' ? 'btn-active' : ''}`}
                  onClick={() => {
                    setEditMode(editMode === 'obstacle' ? null : 'obstacle');
                    setTempObstacle(null);
                    setTrianglePoints([]);
                  }}
                >
                  Add {selectedShape} (−)
                </button>
                <button
                  className="btn btn-danger"
                  onClick={clearObstacles}
                >
                  Clear All Obstacles
                </button>
              </div>
              
              {/* Inflation Radius Slider */}
              <div style={{ marginTop: '16px' }}>
                <div style={{ 
                  display: 'flex', 
                  justifyContent: 'space-between', 
                  alignItems: 'center',
                  marginBottom: '8px'
                }}>
                  <span style={{ color: '#a0a8b0', fontSize: '11px', textTransform: 'uppercase', letterSpacing: '1px' }}>
                    Inflation Radius
                  </span>
                  <span style={{ color: '#ff4a6a', fontSize: '12px', fontWeight: 600 }}>
                    {inflationRadius.toFixed(1)}
                  </span>
                </div>
                <input
                  type="range"
                  min="0"
                  max="5"
                  step="0.5"
                  value={inflationRadius}
                  onChange={(e) => setInflationRadius(parseFloat(e.target.value))}
                  style={{
                    width: '100%',
                    accentColor: '#ff4a6a',
                    cursor: 'pointer'
                  }}
                />
              </div>
              
              {/* Arrow Scale Slider */}
              <div style={{ marginTop: '12px' }}>
                <div style={{ 
                  display: 'flex', 
                  justifyContent: 'space-between', 
                  alignItems: 'center',
                  marginBottom: '8px'
                }}>
                  <span style={{ color: '#a0a8b0', fontSize: '11px', textTransform: 'uppercase', letterSpacing: '1px' }}>
                    Arrow Length
                  </span>
                  <span style={{ color: '#ffc832', fontSize: '12px', fontWeight: 600 }}>
                    {arrowScale.toFixed(1)}x
                  </span>
                </div>
                <input
                  type="range"
                  min="0.5"
                  max="10"
                  step="0.5"
                  value={arrowScale}
                  onChange={(e) => setArrowScale(parseFloat(e.target.value))}
                  style={{
                    width: '100%',
                    accentColor: '#ffc832',
                    cursor: 'pointer'
                  }}
                />
              </div>
              
              {/* Step Size Slider */}
              <div style={{ marginTop: '12px' }}>
                <div style={{ 
                  display: 'flex', 
                  justifyContent: 'space-between', 
                  alignItems: 'center',
                  marginBottom: '8px'
                }}>
                  <span style={{ color: '#a0a8b0', fontSize: '11px', textTransform: 'uppercase', letterSpacing: '1px' }}>
                    Step Size
                  </span>
                  <span style={{ color: '#00ff88', fontSize: '12px', fontWeight: 600 }}>
                    {stepSize.toFixed(2)}
                  </span>
                </div>
                <input
                  type="range"
                  min="0.1"
                  max="2"
                  step="0.1"
                  value={stepSize}
                  onChange={(e) => setStepSize(parseFloat(e.target.value))}
                  style={{
                    width: '100%',
                    accentColor: '#00ff88',
                    cursor: 'pointer'
                  }}
                />
              </div>
              
              {/* Spline Interpolation Checkbox */}
              <div style={{ 
                marginTop: '16px',
                display: 'flex',
                alignItems: 'center',
                gap: '10px'
              }}>
                <input
                  type="checkbox"
                  id="enableSmoothing"
                  checked={enableSmoothing}
                  onChange={(e) => setEnableSmoothing(e.target.checked)}
                  style={{
                    width: '18px',
                    height: '18px',
                    accentColor: '#4a9eff',
                    cursor: 'pointer'
                  }}
                />
                <label 
                  htmlFor="enableSmoothing"
                  style={{ 
                    color: '#a0a8b0', 
                    fontSize: '12px', 
                    cursor: 'pointer',
                    userSelect: 'none'
                  }}
                >
                  Path Smoothing
                </label>
              </div>
              
              {/* Smoothing Iterations Slider - only show when enabled */}
              {enableSmoothing && (
                <div style={{ marginTop: '12px' }}>
                  <div style={{ 
                    display: 'flex', 
                    justifyContent: 'space-between', 
                    alignItems: 'center',
                    marginBottom: '8px'
                  }}>
                    <span style={{ color: '#a0a8b0', fontSize: '11px', textTransform: 'uppercase', letterSpacing: '1px' }}>
                      Smoothness
                    </span>
                    <span style={{ color: '#4a9eff', fontSize: '12px', fontWeight: 600 }}>
                      {smoothingIterations}
                    </span>
                  </div>
                  <input
                    type="range"
                    min="1"
                    max="8"
                    step="1"
                    value={smoothingIterations}
                    onChange={(e) => setSmoothingIterations(parseInt(e.target.value))}
                    style={{
                      width: '100%',
                      accentColor: '#4a9eff',
                      cursor: 'pointer'
                    }}
                  />
                  <div style={{ 
                    display: 'flex', 
                    justifyContent: 'space-between',
                    marginTop: '4px',
                    fontSize: '9px',
                    color: '#6a7a8a'
                  }}>
                    <span>Subtle</span>
                    <span>Smooth</span>
                  </div>
                </div>
              )}
              
              {editMode && (
                <div style={{
                  marginTop: '12px',
                  padding: '10px',
                  background: 'rgba(255, 200, 50, 0.1)',
                  borderRadius: '4px',
                  border: '1px solid rgba(255, 200, 50, 0.3)',
                  color: '#ffc832',
                  fontSize: '11px'
                }}>
                  {editMode === 'obstacle' 
                    ? selectedShape === 'rectangle' 
                      ? 'Click corner 1, then corner 2'
                      : selectedShape === 'circle'
                        ? 'Click center, then drag for radius'
                        : `Click 3 points for triangle (${trianglePoints.length}/3)`
                    : `Click on the grid to place ${editMode}`}
                </div>
              )}
            </div>

            {/* Legend */}
            <div style={{
              background: 'rgba(20, 20, 35, 0.8)',
              borderRadius: '8px',
              padding: '20px',
              border: '1px solid rgba(100, 100, 140, 0.3)'
            }}>
              <h3 style={{
                color: '#a0a8b0',
                fontSize: '12px',
                fontWeight: 600,
                marginBottom: '16px',
                letterSpacing: '2px',
                textTransform: 'uppercase'
              }}>📊 Legend</h3>
              
              <div style={{ display: 'flex', flexDirection: 'column', gap: '10px', fontSize: '12px' }}>
                <div style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
                  <div style={{ width: '20px', height: '20px', background: '#00ff88', borderRadius: '50%' }} />
                  <span style={{ color: '#a0a8b0' }}>Start Point (S)</span>
                </div>
                <div style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
                  <div style={{ width: '20px', height: '20px', background: '#4a9eff', borderRadius: '50%' }} />
                  <span style={{ color: '#a0a8b0' }}>Goal Point (G)</span>
                </div>
                <div style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
                  <div style={{ width: '20px', height: '20px', background: 'rgba(255, 74, 106, 0.5)', border: '2px solid #ff4a6a', borderRadius: '2px' }} />
                  <span style={{ color: '#a0a8b0' }}>Obstacle (− charge)</span>
                </div>
                <div style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
                  <div style={{ width: '24px', height: '24px', border: '1px dashed #ff4a6a', borderRadius: '2px', opacity: 0.6 }} />
                  <span style={{ color: '#a0a8b0' }}>Inflation Zone</span>
                </div>
                <div style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
                  <div style={{ width: '20px', height: '20px', background: '#00ff00', borderRadius: '50%', boxShadow: '0 0 8px #00ff00' }} />
                  <span style={{ color: '#a0a8b0' }}>Active (being pushed)</span>
                </div>
                <div style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
                  <div style={{ width: '16px', height: '16px', background: '#00ffff', border: '2px solid #00ffff', borderRadius: '2px', boxShadow: '0 0 6px #00ffff' }} />
                  <span style={{ color: '#a0a8b0' }}>Locked (won't move)</span>
                </div>
              </div>
            </div>

            {/* Algorithm Info */}
            <div style={{
              background: 'rgba(20, 20, 35, 0.8)',
              borderRadius: '8px',
              padding: '20px',
              border: '1px solid rgba(100, 100, 140, 0.3)'
            }}>
              <h3 style={{
                color: '#a0a8b0',
                fontSize: '12px',
                fontWeight: 600,
                marginBottom: '12px',
                letterSpacing: '2px',
                textTransform: 'uppercase'
              }}>ℹ️ How It Works</h3>
              
              <ol style={{
                color: '#6a7a8a',
                fontSize: '11px',
                lineHeight: '1.6',
                paddingLeft: '16px',
                margin: 0
              }}>
                <li style={{ marginBottom: '6px' }}>Draw straight line S→G</li>
                <li style={{ marginBottom: '6px' }}>Check if any <b style={{color: '#ffc832'}}>segment</b> crosses obstacle</li>
                <li style={{ marginBottom: '6px' }}>Insert point at intersection</li>
                <li style={{ marginBottom: '6px' }}>Push point out, then <b style={{color: '#00ffff'}}>lock it</b></li>
                <li style={{ marginBottom: '6px' }}>Repeat until NO segments cross</li>
              </ol>
            </div>
          </div>
        </div>
        
        {/* Footer - Author & Description */}
        <div style={{
          marginTop: '24px',
          padding: '20px',
          background: 'rgba(20, 20, 35, 0.6)',
          borderRadius: '8px',
          border: '1px solid rgba(100, 100, 140, 0.2)',
          display: 'flex',
          justifyContent: 'space-between',
          alignItems: 'flex-start',
          flexWrap: 'wrap',
          gap: '20px'
        }}>
          {/* Left side - Description */}
          <div style={{ flex: '1', minWidth: '300px' }}>
            <p style={{
              color: '#6a7a8a',
              fontSize: '11px',
              lineHeight: '1.7',
              margin: 0
            }}>
              <b style={{ color: '#a0a8b0' }}>Lazy Coulomb Planner</b> is a reactive path planning algorithm 
              inspired by electrostatic repulsion. It starts with the laziest assumption—a straight line—and 
              only computes when obstacles block the path. Intersection points are pushed away perpendicular 
              to the path direction using Coulomb-like repulsion forces, creating smooth detours around obstacles.
              Best suited for open environments with scattered obstacles.
            </p>
          </div>
          
          {/* Right side - Author */}
          <div style={{ 
            textAlign: 'right',
            minWidth: '200px'
          }}>
            <p style={{
              color: '#4a9eff',
              fontSize: '12px',
              fontWeight: 600,
              margin: '0 0 4px 0'
            }}>
              Developed by Kaveesha Dhananjaya
            </p>
            <p style={{
              color: '#6a7a8a',
              fontSize: '10px',
              margin: 0,
              fontStyle: 'italic'
            }}>
              "Why plan ahead when physics can push you around?"
            </p>
          </div>
        </div>
      </div>
    </div>
  );
};

export default ChargePathPlanner;