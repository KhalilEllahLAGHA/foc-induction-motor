"""Render the English or French project report with Chromium.

Install markdown and pymupdf, then run:
    python tools/make_report_pdf.py
    python tools/make_report_pdf.py --language fr
"""

import argparse
import html
import pathlib
import re
import shutil
import subprocess
import tempfile

import markdown
import pymupdf


ROOT = pathlib.Path(__file__).resolve().parents[1]
CHROME = pathlib.Path(r"C:\Program Files\Google\Chrome\Application\chrome.exe")
EDGE = pathlib.Path(r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe")

REPORTS = {
    "en": {
        "source": "REPORT.md",
        "output": "REPORT.pdf",
        "field": "Electrical machines and drives",
        "kind": "Technical project report",
        "title": "Field-Oriented Control of a Squirrel-Cage Induction Motor",
        "subtitle": "Indirect rotor-flux orientation and an SPWM–SVPWM comparison",
        "author_label": "Prepared by",
        "author": "Khalil Lagha",
        "scope_label": "Simulation study",
        "scope": "MATLAB / Simulink and Python",
        "date": "November 2026",
        "contents": "Contents",
    },
    "fr": {
        "source": "REPORT.fr.md",
        "output": "REPORT.fr.pdf",
        "field": "Machines électriques et entraînements",
        "kind": "Compte rendu de projet",
        "title": "Commande vectorielle d’un moteur asynchrone à cage",
        "subtitle": "Orientation indirecte du flux rotorique et comparaison SPWM–SVPWM",
        "author_label": "Réalisé par",
        "author": "Lagha Khalil",
        "scope_label": "Étude en simulation",
        "scope": "MATLAB / Simulink et Python",
        "date": "Novembre 2026",
        "contents": "Sommaire",
    },
}


def read_body(source):
    text = source.read_text(encoding="utf-8")
    first_section = re.search(r"^## ", text, flags=re.M)
    if first_section is None:
        raise ValueError(f"No report sections in {source}")
    math = []

    def stash(match):
        math.append(match.group())
        return f"XMATHX{len(math) - 1}XENDX"

    body = re.sub(r"\$\$.*?\$\$|\$[^$\n]+?\$", stash, text[first_section.start():], flags=re.S)
    renderer = markdown.Markdown(extensions=["toc", "tables", "fenced_code"])
    body = renderer.convert(body)
    body = re.sub(r"XMATHX(\d+)XENDX", lambda m: math[int(m[1])], body)
    for relative in ("python/outputs/graphs", "matlab/outputs/graphs", "matlab/outputs/diagrams"):
        body = body.replace(f'src="{relative}/', f'src="{(ROOT / relative).as_uri()}/')
    return body, renderer.toc_tokens


def contents_list(sections, pages, nested=False):
    rows = []
    for section in sections:
        anchor = html.escape(section["id"], quote=True)
        label = html.escape(html.unescape(section["name"]))
        number = pages.get(section["id"], "00")
        child = contents_list(section["children"], pages, nested=True) if section["children"] else ""
        rows.append(
            f'<li><a href="#{anchor}"><span>{label}</span><span class="leader"></span>'
            f'<span class="toc-page">{number}</span></a>{child}</li>'
        )
    kind = "toc-sub" if nested else "toc-sections"
    return f'<ol class="{kind}">' + "\n".join(rows) + "</ol>"


def report_html(language, labels, body, sections, pages):
    return f"""<!doctype html>
<html lang="{language}"><head><meta charset="utf-8">
<title>{labels['title']}</title>
<style>
@page {{
  size: A4;
  margin: 20mm;
  @top-left {{ content: "{labels['field']}"; font: italic 9pt 'Times New Roman'; }}
  @bottom-center {{ content: counter(page); width: 170mm; border-top: 1pt solid #222;
                   padding-top: 2mm; vertical-align: top; font: 10pt 'Times New Roman'; }}
}}
@page cover {{
  @top-left {{ content: none; }}
  @bottom-center {{ content: none; border: none; }}
}}
@page contents {{
  @top-left {{ content: none; }}
}}
* {{ box-sizing: border-box; }}
body {{ margin: 0; color: #111; font: 14pt/1.32 'Times New Roman', serif; }}
.cover {{ page: cover; break-after: page; padding-top: 12mm; text-align: center; }}
.cover-field {{ font-style: italic; font-size: 14pt; padding: 5mm 0 7mm; border-bottom: 1pt solid; }}
.cover-kind {{ margin: 36mm 0 11mm; font-weight: bold; font-size: 19pt; }}
.title-block {{ padding: 10mm 4mm; border-block: 1.2pt solid; }}
.title-block h1 {{ font-size: 25pt; line-height: 1.18; margin: 0 0 5mm; }}
.title-block p {{ margin: 0; font-size: 15pt; text-align: center; }}
.cover-info {{ display: grid; grid-template-columns: 1fr 1fr; gap: 12mm; text-align: left;
              margin: 20mm 9mm 17mm; }}
.cover-info p {{ margin: 2mm 0 0; text-align: left; }}
.cover-date {{ padding-top: 7mm; border-top: 1pt solid; font-size: 14pt; }}
.contents {{ page: contents; break-after: page; padding-top: 3mm; }}
.contents h1 {{ font-size: 22pt; margin: 0 0 9mm; }}
.contents ol {{ list-style: none; padding-left: 0; margin: 0; }}
.toc-sections > li {{ margin: 2.8mm 0; }}
.toc-sub {{ margin: 1mm 0 2mm 5mm !important; font-size: 11.5pt; }}
.toc-sub li {{ margin: 0.7mm 0; }}
.contents a {{ color: #111; text-decoration: none; display: flex; align-items: baseline; gap: 2mm; }}
.toc-sections > li > a {{ font-weight: bold; }}
.leader {{ flex: 1; min-width: 3mm; border-bottom: 1px dotted #555; }}
.toc-page {{ min-width: 7mm; text-align: right; }}
h2 {{ font-size: 19pt; margin: 9mm 0 3mm; break-after: avoid; }}
main > h2:first-child {{ margin-top: 0; }}
h3 {{ font-size: 16pt; margin: 6mm 0 2.5mm; break-after: avoid; }}
p {{ text-align: justify; margin: 0 0 3mm; widows: 2; orphans: 2; }}
main a {{ color: #154777; text-decoration-thickness: 0.07em; }}
p:has(> img) {{ break-inside: avoid; }}
img {{ max-width: 100%; display: block; margin: 4mm auto; break-inside: avoid; }}
img[src$="foc_architecture.svg"] {{ width: 100%; }}
img[src$="svpwm_hexagon.png"] {{ width: 76%; }}
img[src$="library_top.png"], img[src$="blocks_speed.png"],
img[src$="blocks_dc_link.png"] {{ width: 85%; }}
img[src$="library_pwm_comparison.png"], img[src$="library_pwm_zoom.png"] {{ width: 91%; }}
table {{ border-collapse: collapse; width: 100%; margin: 4mm 0; font-size: 11pt; break-inside: avoid; }}
th, td {{ border: 1px solid #aaa; padding: 1.6mm 2mm; text-align: left; }}
th {{ background: #f2f2f2; }}
code {{ font: 0.87em Consolas, monospace; }}
li {{ margin: 1mm 0; }}
mjx-container {{ break-inside: avoid; }}
</style>
<script>
window.MathJax = {{tex: {{inlineMath: [['$', '$']], displayMath: [['$$', '$$']]}},
                   chtml: {{scale: 0.94}}}};
</script>
<script src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-chtml.js"></script>
</head><body>
<section class="cover">
  <div class="cover-field">{labels['field']}</div>
  <div class="cover-kind">{labels['kind']}</div>
  <div class="title-block"><h1>{labels['title']}</h1><p>{labels['subtitle']}</p></div>
  <div class="cover-info">
    <div><strong>{labels['author_label']}</strong><p>{labels['author']}</p></div>
    <div><strong>{labels['scope_label']}</strong><p>{labels['scope']}</p></div>
  </div>
  <div class="cover-date">{labels['date']}</div>
</section>
<nav class="contents" aria-label="{labels['contents']}">
  <h1>{labels['contents']}</h1>{contents_list(sections, pages)}
</nav>
<main>{body}</main>
</body></html>
"""


def print_pdf(browser, work, source, destination):
    result = subprocess.run(
        [browser, "--headless=new", "--disable-gpu", "--no-first-run", "--no-pdf-header-footer",
         f"--user-data-dir={work / 'browser-profile'}", "--virtual-time-budget=30000",
         "--run-all-compositor-stages-before-draw", f"--print-to-pdf={destination}",
         source.as_uri()],
        capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=120,
    )
    if result.returncode or not destination.is_file() or not destination.stat().st_size:
        raise RuntimeError(f"Chromium did not render {destination.name}: {result.stderr[-4000:]}")


def section_pages(pdf):
    with pymupdf.open(pdf) as document:
        return {link["nameddest"]: link["page"] + 1 for link in document[1].get_links()
                if link.get("nameddest") and link.get("page", -1) >= 2}


def section_ids(sections):
    for section in sections:
        yield section["id"]
        yield from section_ids(section["children"])


def main():
    args = argparse.ArgumentParser(description=__doc__)
    args.add_argument("--language", choices=REPORTS, default="en")
    args.add_argument("--browser", default=None, help="Optional path to a Chromium executable")
    options = args.parse_args()
    browser = options.browser or next((str(p) for p in (CHROME, EDGE) if p.is_file()), None)
    browser = browser or shutil.which("chromium") or shutil.which("google-chrome")
    if not browser:
        raise SystemExit("Install Chrome or Chromium, or specify --browser.")

    labels = REPORTS[options.language]
    body, sections = read_body(ROOT / labels["source"])
    expected = set(section_ids(sections))
    with tempfile.TemporaryDirectory(prefix="foc-report-") as directory:
        work = pathlib.Path(directory)
        source = work / "report.html"
        draft = work / "draft.pdf"
        final = work / "final.pdf"
        source.write_text(report_html(options.language, labels, body, sections, {}), encoding="utf-8")
        print_pdf(browser, work, source, draft)
        pages = section_pages(draft)
        if set(pages) != expected:
            raise RuntimeError(f"Contents targets missing from draft: {sorted(expected - set(pages))}")

        source.write_text(report_html(options.language, labels, body, sections, pages), encoding="utf-8")
        print_pdf(browser, work, source, final)
        if section_pages(final) != pages:
            raise RuntimeError("Section pages changed after inserting contents numbers.")

        with pymupdf.open(final) as document:
            text = "".join(page.get_text() for page in document)
            if "$$" in text or r"\begin{bmatrix}" in text:
                raise RuntimeError("Report equations did not render.")
            metadata = document.metadata
            metadata.update(title=labels["title"], author=labels["author"], subject=labels["subtitle"])
            document.set_metadata(metadata)
            output = ROOT / labels["output"]
            document.save(output)
            print(f"{output.name} · {len(document)} pages")


if __name__ == "__main__":
    main()
