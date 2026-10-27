# Writing the documentation

The documentation lives **in the repository**, next to the code (*docs as
code*). A change to the firmware and the change to its page go in the same
pull request and are reviewed together.

## Where things go

| What | Where | Tool |
|---|---|---|
| Theory, architecture, procedures, guides | `03 - Docs/**/*.md` | MkDocs Material, published on GitHub Pages |
| Why a design is the way it is | `03 - Docs/decisions/NNNN-*.md` | ADR ([template](../decisions/index.md)) |
| C API: functions, structs, constants | comments in the headers | Doxygen + Graphviz ([API reference](../api/index.md)) |
| Diagrams | inside the page, as a ```` ```mermaid ```` block | Mermaid (rendered by GitHub and by the site) |
| Equations | `$…$` inline, `$$…$$` display | MathJax on the site, native on GitHub |
| How to build or run one folder | a short `README.md` in that folder, with a link to its page | – |
| Field notes, bench logs, flight tests | the GitHub Wiki | not tied to a code version |

The pages use only standard Markdown, `$math$` and Mermaid, so they read the
same on GitHub and on the site. Avoid syntax that only one of the two renders
(for example MkDocs admonitions `!!! note`).

## Build and preview

```
pip install -r "03 - Docs/requirements.txt"
doxygen Doxyfile.fc && doxygen Doxyfile.flow    # optional locally; the CI always does it
mkdocs serve                                    # http://127.0.0.1:8000, reloads on save
mkdocs build --strict                           # what the CI runs
```

`--strict` turns every warning into an error. That includes a link to a
missing page, an anchor that does not exist, and a page missing from `nav`. A
broken cross-reference therefore cannot be published.

## Publishing

`.github/workflows/docs.yml` runs on every push to `main` that touches the
documentation or the sources:

1. it installs Doxygen, Graphviz and the pinned MkDocs packages;
2. it generates both API references;
3. it builds the site with `--strict`;
4. it publishes the site to GitHub Pages.

On pull requests it only builds, so a broken page shows up in the review.

**One-time setup:** in the repository settings, open *Pages* and set the
*Source* to *GitHub Actions*.

## Conventions

- **One topic per page.** A page longer than a few screens is split, and
  links replace section numbers. Links survive reordering; "§12.5" does not.
- **Links are relative** between pages (`../theory/iekf.md#anchor`). Files
  outside `03 - Docs` are linked with their GitHub URL, so the link works in
  both places.
- **Equations in the form used by the code**, with the name of the file that
  implements them.
- **Diagrams as code** (Mermaid), never as screenshots. They change in the same
  diff as the text.
- **Honest status.** What runs on the board, what is host-tested only, and
  what is missing are written down as such.
- English, short sentences, units always given.
