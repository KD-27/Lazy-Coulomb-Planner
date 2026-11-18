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
