# Complete English engineering report

The English edition translates the complete Chinese v1.2 report: ten chapters, four appendices, references, all 29 stages, narrative, tables, figure captions, and code/video indexes. Experimental claims, values, units, source attribution, and limitations are retained. This is a translated edition, not a new experiment or an abridged report.

- PDF: [build/main_en.pdf](build/main_en.pdf)
- LaTeX entry: [main_en.tex](main_en.tex)
- Chinese edition: [../build/main_zh.pdf](../build/main_zh.pdf)
- Playable videos: [GitHub video gallery](../../../docs/DEMOS.md), including [official G1 RL validation](../../../docs/UNITREE_RL_DEMOS.md)

The two editions share the original figures and videos in `../figures/` and `../videos/`. Embedded text in original screenshots and recorded experiment frames is retained as part of the original records; captions and surrounding explanations are translated. Video filenames and repository paths are unchanged. The English PDF has its own pagination.

Compile from this directory using `./build.sh` with XeLaTeX/latexmk or Tectonic. Times New Roman is used for the text, headings, and code; the mathematical typeface is Times-compatible. Install Times New Roman locally before compiling. Font files are not redistributed with the repository. No model or GPU is needed to compile or read the report. Keep this directory beside the shared figures and videos.

Validation: `python3 validate.py` uses Python 3 and `pdfinfo` from Poppler to compare the two editions and check media coverage, references, fonts, and PDF metadata. See `VALIDATION.json` and `TRANSLATION_CHECK.json` for the release checks.
