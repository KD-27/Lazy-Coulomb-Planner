# Reference check (18 November 2026)

Every entry of `refs.bib` was compared with the publisher's record: Crossref for 40 DOIs
(title, authors incl. given names, venue, volume, issue, pages, year), DataCite via doi.org for
Udupa's thesis and the Amanatides-Woo paper, arXiv for R2+, GitHub for the three software entries.

Result: no wrong title, author, volume, issue, page range or year.

Changes made:
- DOIs added to Hart et al. 1968, Pohl 1970, Chaikin 1974, Amanatides & Woo 1987 (records match
  the entries exactly); Amanatides & Woo booktitle now "Eurographics 1987 -- Technical Papers".
- jps3d: year 2019 (last commit ef65e36; README names the library v1.1, matching the note).
- Earlier the same day: `{R2}` and `{Anya}` braced.

Checked and left as they are:
- Ginés Clavero (Marathon 2) and Nagaratnam Suganthan (APPATT): Crossref splits these
  compound names differently; the bib follows the authors' own usage.
- Uras & Koenig, SoCS: Crossref dates the AAAI digital edition 2021; the symposium and the
  paper are 2015.
- R2+ (arXiv:2405.04952): no journal version found on Crossref as of today; recheck before submission.
- Smac (RA-P, 2026) was verified on 17 November.
- `borenstein1991vfh` is not cited; BibTeX does not print it.

Not checked: whether each sentence that cites a paper states its content correctly (that needs
the papers themselves).

## Second check: does each citing sentence describe its source? (18 November 2026)

After an external review that read the sources. Confirmed and fixed:
- Smac [macenski2024smac]: only 2D-A* takes over a second on the warehouse map (1358 ms; Hybrid-A*
  290 ms, State Lattice 473 ms, arXiv 2401.13078v2). Text now says "for Smac 2D". Volume 1,
  pages 81-85 and the DOI are on Crossref, so the entry stays.
- R2/R2+ work on binary occupancy grids, not polygons (only RayScan uses polygons): reworded.
- R2 has a public C++ implementation (github.com/LaiYanKai/2D, no licence file, last push
  May 2024); R2+ code is not published. "No public implementation" claim corrected; new
  entry `r2code`.
- EGO-Planner picks the push direction from an A* guiding path per colliding segment
  (arXiv 2008.08835): added.
- jps3d DMPlanner searches again in a region around the JPS path; it does not move the path:
  reworded.
- Amanatides-Woo: the four-cell corner rule is our extension: "extended so that".
- Nav2's Theta* is Lazy Theta*-P (Nav2 README): `nash2010lazy` added (Crossref checked).
- AMCL: `fox2003kld` added (Crossref checked). TEB: original `rosmann2012teb` added next to the
  2017 multi-topology paper; not on Crossref or dblp, so its pages are not given and still
  need checking against the ROBOTIK 2012 proceedings.
