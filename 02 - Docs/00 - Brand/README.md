# ACwing – brand files

**Project:** ACwing · **Author:** Andrea Cirillo · **Overview:** [repository README](../../README.md)

<p>
  <img src="ACwing_logo_on_black.png" alt="ACwing logo on black" width="220">
  <img src="ACwing_logo_black.png" alt="ACwing logo, black" width="220">
</p>

These are the ACwing logos used by the documentation of this repository. They are copies of the files in the
hardware repository, which holds the master vector source and the tools that generate them:
[`ACwing_FlightController_HW` → `00 - Docs/05 - Brand`](https://github.com/andrea-cirillo/ACwing_FlightController_HW/tree/main/00%20-%20Docs/05%20-%20Brand).
Change the logos there, then copy them here, so both repositories keep the same files.

| File | Use |
|---|---|
| `ACwing_logo_black.svg / .png` | full logo (mark + wordmark), black on transparent: light backgrounds, documents |
| `ACwing_logo_white.svg / .png` | full logo, white on transparent: dark backgrounds |
| `ACwing_logo_on_black.svg / .png` | full logo, white on a black square: avatars, covers |
| `ACwing_mark_black.svg / .png`, `ACwing_mark_white.svg / .png` | mark only (drone + "AC" monogram): small spaces, icons |

SVG is the master format, since it scales without loss. The PNGs are 1024 px wide, with transparency.

The READMEs show the black and white PNGs as a pair, inside a `<picture>` element, so that GitHub shows the right
one in light mode and in dark mode:

```html
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="02%20-%20Docs/00%20-%20Brand/ACwing_logo_white.png">
  <img alt="ACwing" src="02%20-%20Docs/00%20-%20Brand/ACwing_logo_black.png" width="260">
</picture>
```
