# How To Run - Lazy Coulomb Planner

This guide explains how to run the Lazy Coulomb Planner visualization on your machine.

---

## Prerequisites

- **Node.js** (version 16 or higher) - [Download here](https://nodejs.org/)
- **npm** (comes with Node.js) or **yarn**

To check if you have Node.js installed:
```bash
node --version
npm --version
```

---

## Option 1: Quick Start (Online - No Installation)

### Using CodeSandbox
1. Go to [codesandbox.io](https://codesandbox.io)
2. Click **Create Sandbox** → Select **React** (Vite)
3. In the file explorer, open `src/App.jsx`
4. Delete all content and paste the code from `lazy-coulomb-planner.jsx`
5. The preview will automatically update

### Using StackBlitz
1. Go to [stackblitz.com](https://stackblitz.com)
2. Click **Start a new project** → Select **React**
3. Open `src/App.jsx`
4. Replace content with code from `lazy-coulomb-planner.jsx`
5. Preview updates automatically

---

## Option 2: Local Setup with Vite (Recommended)

Vite is the fastest way to run React locally.

### Step 1: Create Project
```bash
npm create vite@latest lazy-coulomb-planner -- --template react
```

### Step 2: Navigate to Project
```bash
cd lazy-coulomb-planner
```

### Step 3: Install Dependencies
```bash
npm install
```

### Step 4: Replace App Component

Copy the contents of `lazy-coulomb-planner.jsx` and replace `src/App.jsx`:

**On macOS/Linux:**
```bash
cp /path/to/lazy-coulomb-planner.jsx src/App.jsx
```

**On Windows:**
```bash
copy \path\to\lazy-coulomb-planner.jsx src\App.jsx
```

Or manually:
1. Open `src/App.jsx` in your editor
2. Delete all existing content
3. Paste the code from `lazy-coulomb-planner.jsx`

### Step 5: Update Main Entry (if needed)

Make sure `src/main.jsx` imports correctly:

```jsx
import React from 'react'
import ReactDOM from 'react-dom/client'
import App from './App.jsx'
import './index.css'

ReactDOM.createRoot(document.getElementById('root')).render(
  <React.StrictMode>
    <App />
  </React.StrictMode>,
)
```

### Step 6: Run Development Server
```bash
npm run dev
```

### Step 7: Open in Browser

Navigate to:
```
http://localhost:5173
```

---

## Option 3: Local Setup with Create React App

### Step 1: Create Project
```bash
npx create-react-app lazy-coulomb-planner
```

### Step 2: Navigate to Project
```bash
cd lazy-coulomb-planner
```

### Step 3: Replace App Component

Replace `src/App.js` with the contents of `lazy-coulomb-planner.jsx`:

```bash
# macOS/Linux
cp /path/to/lazy-coulomb-planner.jsx src/App.js

# Windows
copy \path\to\lazy-coulomb-planner.jsx src\App.js
```

### Step 4: Run Development Server
```bash
npm start
```

### Step 5: Open in Browser

Navigate to:
```
http://localhost:3000
```

---

## Option 4: Using Existing React Project

If you already have a React project:

### Step 1: Copy the Component File
```bash
cp lazy-coulomb-planner.jsx src/components/LazyCoulombPlanner.jsx
```

### Step 2: Import and Use

In your `App.jsx` or any parent component:

```jsx
import LazyCoulombPlanner from './components/LazyCoulombPlanner';

function App() {
  return (
    <div>
      <LazyCoulombPlanner />
    </div>
  );
}

export default App;
```

---

## Project Structure

After setup, your project should look like:

```
lazy-coulomb-planner/
├── node_modules/
├── public/
├── src/
│   ├── App.jsx          ← Lazy Coulomb Planner code goes here
│   ├── main.jsx         ← Entry point
│   └── index.css        ← Global styles (optional)
├── package.json
├── vite.config.js       ← (if using Vite)
└── README.md
```

---

## Troubleshooting

### Error: "Module not found"
Make sure the export/import names match:
```jsx
// At the end of lazy-coulomb-planner.jsx
export default ChargePathPlanner;

// In main.jsx
import App from './App.jsx'  // This imports ChargePathPlanner as App
```

### Error: "React is not defined"
Add React import at the top of the file:
```jsx
import React, { useState, useEffect, useCallback } from 'react';
```

### Blank Screen
1. Open browser Developer Tools (F12)
2. Check the Console tab for errors
3. Ensure all dependencies are installed: `npm install`

### Port Already in Use
```bash
# Kill process on port 5173 (Vite)
npx kill-port 5173

# Or use a different port
npm run dev -- --port 3001
```

### Styling Issues
The component uses inline styles, so no additional CSS is required. If styles look wrong:
1. Clear browser cache (Ctrl+Shift+R)
2. Ensure no conflicting global CSS

---

## Build for Production

To create an optimized production build:

```bash
# Using Vite
npm run build

# Output will be in dist/ folder
```

To preview the production build:
```bash
npm run preview
```

---

## Deployment

### Deploy to Vercel
```bash
npm install -g vercel
vercel
```

### Deploy to Netlify
```bash
npm run build
# Drag and drop dist/ folder to netlify.com
```

### Deploy to GitHub Pages
```bash
npm install gh-pages --save-dev
```

Add to `package.json`:
```json
{
  "homepage": "https://yourusername.github.io/lazy-coulomb-planner",
  "scripts": {
    "predeploy": "npm run build",
    "deploy": "gh-pages -d dist"
  }
}
```

Then run:
```bash
npm run deploy
```

---

## System Requirements

| Requirement | Minimum | Recommended |
|-------------|---------|-------------|
| Node.js | 16.x | 18.x or 20.x |
| RAM | 2 GB | 4 GB+ |
| Browser | Chrome 90+ | Latest Chrome/Firefox/Edge |
| Screen | 1024×768 | 1920×1080+ |

---

## Quick Commands Reference

| Action | Command |
|--------|---------|
| Create project (Vite) | `npm create vite@latest project-name -- --template react` |
| Install dependencies | `npm install` |
| Run dev server | `npm run dev` |
| Build for production | `npm run build` |
| Preview production build | `npm run preview` |

---

## Need Help?

If you encounter issues:
1. Ensure Node.js is version 16+
2. Delete `node_modules` and run `npm install` again
3. Check browser console for errors
4. Ensure you're using a modern browser

---

*Happy Path Planning! 🦥⚡*
