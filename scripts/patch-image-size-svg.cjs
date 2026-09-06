#!/usr/bin/env node
/**
 * Patch image-size's SVG detector to scan the whole file instead of only the
 * first 1000 bytes.
 *
 * Why: Docusaurus (mdx-loader/transformImage) uses image-size to inject
 * width/height attributes on markdown images. image-size's SVG "validate"
 * fast-path only looks at the first 1000 bytes of a file, but draw.io SVG
 * exports embed the entire editable mxfile XML in a huge `content` attribute
 * at the front of the root <svg> tag. That pushes the root tag's closing '>'
 * well past byte 1000 (typically byte 2.7k-73k), so detection fails with
 * "unsupported file type: undefined" and Docusaurus prints a bogus
 * "The image ... can't be read correctly" warning - even though the SVG is
 * perfectly valid and image-size's own calculate() parses it fine.
 *
 * Fix: make validate() scan the whole (<=512KB, image-size caps input size)
 * input, exactly like calculate() already does. No behavior change for small
 * SVGs; matches upstream intention (the "first kilo-byte" was just a speed
 * optimization that breaks legitimately large root tags).
 *
 * Runs from package.json "postinstall" so it re-applies automatically after
 * every fresh install / npm ci (e.g. in CI).
 */
'use strict';

const fs = require('fs');
const path = require('path');

const candidates = [
  path.join(__dirname, '..', 'node_modules', 'image-size', 'dist', 'types', 'svg.js'),
  path.join(__dirname, '..', 'node_modules', 'image-size', 'dist', 'types', 'svg.cjs'),
  path.join(__dirname, '..', 'node_modules', 'image-size', 'dist', 'types', 'svg.mjs'),
];

// image-size 1.x compiled text (what Docusaurus 3.7.0's lockfile resolves to)
const OLD_1X = '(0, utils_1.toUTF8String)(input, 0, 1000)';
const NEW_1X = '(0, utils_1.toUTF8String)(input)';
// The patched validate() line, e.g.:
//   (0, utils_1.toUTF8String)(input).test ...
// Actually the compiled line is: validate: (input) => svgReg.test((0, utils_1.toUTF8String)(input)),
const PATCHED_VALIDATE_MARKER = 'validate: (input) => svgReg.test((0, utils_1.toUTF8String)(input))';

function validateIsPatched(src) {
  // Recognize a validate line that scans the whole input.
  const lines = src.split('\n');
  const idx = lines.findIndex((l) => l.includes('validate: (input) => svgReg.test'));
  if (idx === -1) {
    return false; // not the layout we know
  }
  const line = lines[idx];
  // patched: ...toUTF8String)(input)) with no ", 0, 1000" cap
  return line.includes('toUTF8String)(input)') && !line.includes('toUTF8String)(input, 0, 1000)');
}

let found = 0;
let patched = 0;

for (const file of candidates) {
  if (!fs.existsSync(file)) continue;
  found += 1;
  let src = fs.readFileSync(file, 'utf8');

  if (src.includes(OLD_1X)) {
    const occurrences = src.split(OLD_1X).length - 1;
    if (occurrences !== 1) {
      console.warn(
        '[patch-image-size-svg] WARNING: ' + file +
        ' contains ' + occurrences + ' instances of the validate scan; expected 1. Skipping to stay safe.',
      );
      continue;
    }
    src = src.replace(OLD_1X, NEW_1X);
    fs.writeFileSync(file, src);
    patched += 1;
    console.log('[patch-image-size-svg] patched ' + path.relative(process.cwd(), file));
  } else if (validateIsPatched(src)) {
    console.log('[patch-image-size-svg] already patched ' + path.relative(process.cwd(), file));
    patched += 1;
  } else {
    console.warn(
      '[patch-image-size-svg] WARNING: ' + file +
      ' does not match a known image-size SVG validate layout; the draw.io SVG warning may come back. ' +
      'Please update this patch script.',
    );
  }
}

if (found === 0) {
  console.warn(
    '[patch-image-size-svg] WARNING: image-size package not found under node_modules; ' +
    'nothing to patch. (During "npm ci" this is not an error on its own.)',
  );
} else if (patched === 0) {
  console.warn('[patch-image-size-svg] WARNING: no image-size SVG handler was patchable.');
}

process.exit(0);
