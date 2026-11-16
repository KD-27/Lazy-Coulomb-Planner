# Paper draft

`main.tex` is a draft of a paper on the Lazy Coulomb Planner, in the IEEE
conference format.

## Viewing it in Overleaf

1. In Overleaf: New Project, Upload Project, and upload this whole folder as a zip
   (`main.tex`, `refs.bib` and the `figures/` folder with its PDF files).
2. Menu: Compiler `pdfLaTeX`, main document `main.tex`.
3. Recompile. The IEEEtran class and all packages used are part of Overleaf.

## State of the draft

- Numbers come from the experiments in `../experiments/2026-11-15_v2.1.0/`
  (planner v2.1.0, optimised build).
- Text in red, `\pending{...}`, is waiting for results still being collected or
  for a decision by the authors: affiliation, co-authors, related work, the
  driving results and the comfort-cost ablation.
- `refs.bib` was written from memory: check every entry against the original
  publication before submission.
- The target venue, and therefore the page limit and format, are not decided;
  the IEEE conference class is a placeholder that is easy to change.

## Figures

`figures/make_figures.py` regenerates the PDF figures from the benchmark data:

```bash
python3 paper/figures/make_figures.py     # from the repository root
```
