#!/usr/bin/env node

import fs from 'fs';
import path from 'path';
import { spawnSync } from 'child_process';

const args = process.argv.slice(2);

function getArg(name, fallback) {
  const index = args.indexOf(name);
  return index === -1 || !args[index + 1] ? fallback : args[index + 1];
}

function hasFlag(name) {
  return args.includes(name);
}

const buildDir = path.resolve(getArg('--build-dir', './build'));
const maxSize = Number(getArg('--max-size', '25')) * 1024 * 1024;
const gifTimeoutMs = Number(getArg('--gif-timeout', '180')) * 1000;
const imageTimeoutMs = Number(getArg('--image-timeout', '120')) * 1000;
const onlyGif = hasFlag('--only-gif');
const rewriteReferences = hasFlag('--rewrite-references');
const removeUnsupported = hasFlag('--remove-unsupported');
const removeRemaining = hasFlag('--remove-remaining');

const textExtensions = new Set([
  '.css',
  '.html',
  '.js',
  '.json',
  '.map',
  '.svg',
  '.txt',
  '.xml',
]);

function formatSize(bytes) {
  return `${(bytes / 1024 / 1024).toFixed(2)} MB`;
}

function walkFiles(dir) {
  const files = [];
  for (const entry of fs.readdirSync(dir, { withFileTypes: true })) {
    const filePath = path.join(dir, entry.name);
    if (entry.isDirectory()) {
      files.push(...walkFiles(filePath));
    } else if (entry.isFile()) {
      files.push(filePath);
    }
  }
  return files;
}

function fileSize(filePath) {
  return fs.statSync(filePath).size;
}

function commandExists(command) {
  return spawnSync('bash', ['-lc', `command -v ${command}`], {
    stdio: 'ignore',
  }).status === 0;
}

function run(command, commandArgs, timeoutMs) {
  const result = spawnSync(command, commandArgs, {
    stdio: 'inherit',
    timeout: timeoutMs,
  });

  if (result.error) {
    if (result.error.code === 'ETIMEDOUT') {
      console.error(`[asset] ${command} timed out after ${timeoutMs / 1000}s`);
    } else {
      console.error(`[asset] ${command} failed: ${result.error.message}`);
    }
    return false;
  }

  return result.status === 0;
}

function replaceReferences(oldName, newName) {
  if (!rewriteReferences) return;

  let changedFiles = 0;
  for (const filePath of walkFiles(buildDir)) {
    if (!textExtensions.has(path.extname(filePath).toLowerCase())) continue;

    let content;
    try {
      content = fs.readFileSync(filePath, 'utf8');
    } catch {
      continue;
    }

    if (!content.includes(oldName)) continue;

    const nextContent = content.split(oldName).join(newName);
    fs.writeFileSync(filePath, nextContent);
    changedFiles += 1;
  }

  console.log(`[asset] rewrote ${changedFiles} file(s): ${oldName} -> ${newName}`);
}

function safeUnlink(filePath) {
  try {
    fs.unlinkSync(filePath);
  } catch (error) {
    if (error.code !== 'ENOENT') throw error;
  }
}

function convertGif(filePath) {
  const outputPath = filePath.replace(/\.gif$/i, '.webp');
  safeUnlink(outputPath);

  const ok = run(
    'ffmpeg',
    [
      '-hide_banner',
      '-loglevel',
      'error',
      '-i',
      filePath,
      '-vcodec',
      'libwebp',
      '-lossless',
      '0',
      '-q:v',
      '75',
      '-loop',
      '0',
      '-an',
      '-vsync',
      '0',
      outputPath,
      '-y',
    ],
    gifTimeoutMs,
  );

  if (ok && fs.existsSync(outputPath) && fileSize(outputPath) > 0) {
    console.log(
      `[asset] converted ${path.relative(buildDir, filePath)} (${formatSize(fileSize(filePath))}) -> ${path.basename(outputPath)} (${formatSize(fileSize(outputPath))})`,
    );
    replaceReferences(path.basename(filePath), path.basename(outputPath));
    safeUnlink(filePath);
    return true;
  }

  safeUnlink(outputPath);
  console.error(`[asset] failed to convert ${path.relative(buildDir, filePath)}`);
  if (removeUnsupported) {
    safeUnlink(filePath);
    console.error(`[asset] removed ${path.relative(buildDir, filePath)}`);
    return true;
  }
  return false;
}

function optimizeRaster(filePath) {
  if (!commandExists('convert')) {
    console.error('[asset] ImageMagick convert is not available');
    return false;
  }

  const ext = path.extname(filePath);
  const outputPath = `${filePath}.optimized${ext}`;
  safeUnlink(outputPath);

  const ok = run(
    'convert',
    [filePath, '-quality', '80', '-resize', '4096x4096>', outputPath],
    imageTimeoutMs,
  );

  if (ok && fs.existsSync(outputPath) && fileSize(outputPath) > 0) {
    const nextSize = fileSize(outputPath);
    if (nextSize <= maxSize) {
      fs.renameSync(outputPath, filePath);
      console.log(`[asset] optimized ${path.relative(buildDir, filePath)} (${formatSize(nextSize)})`);
      return true;
    }
    console.error(`[asset] optimized file is still too large: ${path.relative(buildDir, filePath)} (${formatSize(nextSize)})`);
  }

  safeUnlink(outputPath);
  return false;
}

function handleOversized(filePath) {
  const ext = path.extname(filePath).toLowerCase();

  if (ext === '.gif') {
    return convertGif(filePath);
  }

  if (onlyGif) {
    return false;
  }

  if (ext === '.png' || ext === '.jpg' || ext === '.jpeg') {
    if (optimizeRaster(filePath)) return true;
  }

  if (removeUnsupported) {
    safeUnlink(filePath);
    console.error(`[asset] removed unsupported oversized file: ${path.relative(buildDir, filePath)}`);
    return true;
  }

  return false;
}

if (!fs.existsSync(buildDir)) {
  console.error(`[asset] build directory not found: ${buildDir}`);
  process.exit(1);
}

const oversizedFiles = walkFiles(buildDir).filter((filePath) => fileSize(filePath) > maxSize);
const filesToProcess = oversizedFiles.filter((filePath) => {
  if (!onlyGif) return true;
  return path.extname(filePath).toLowerCase() === '.gif';
});

if (filesToProcess.length === 0) {
  console.log('[asset] no oversized assets to process');
  process.exit(0);
}

for (const filePath of filesToProcess) {
  console.log(`[asset] processing ${path.relative(buildDir, filePath)} (${formatSize(fileSize(filePath))})`);
  handleOversized(filePath);
}

if (removeRemaining) {
  const remaining = walkFiles(buildDir).filter((filePath) => fileSize(filePath) > maxSize);
  for (const filePath of remaining) {
    console.error(`[asset] removing remaining oversized file: ${path.relative(buildDir, filePath)} (${formatSize(fileSize(filePath))})`);
    safeUnlink(filePath);
  }
}

const finalOversized = walkFiles(buildDir).filter((filePath) => {
  if (fileSize(filePath) <= maxSize) return false;
  if (!onlyGif) return true;
  return path.extname(filePath).toLowerCase() === '.gif';
});

if (finalOversized.length > 0 && !onlyGif) {
  console.error('[asset] oversized files remain:');
  for (const filePath of finalOversized) {
    console.error(`  - ${path.relative(buildDir, filePath)} (${formatSize(fileSize(filePath))})`);
  }
  process.exit(1);
}

console.log('[asset] large asset processing complete');
