#!/usr/bin/env node
'use strict';
// ──────────────────────────────────────────────────────────────────────────────
// simulator-sync.js  —  keeps simulator.html in sync with autonomous.cpp
//
// HOW TO START:
//   node simulator-sync.js          ← watch mode: re-syncs every time a
//                                     watched C++ file is saved
//   node simulator-sync.js --once   ← sync once and exit (useful for CI or
//                                     a one-off manual refresh)
//
// You do NOT need npm install — this script only uses Node.js built-ins
// (fs, path).  Any Node.js version >= 12 works.
//
// WHAT IT DOES (4 steps on each sync):
//   1. Reads user.cpp to find the current auton_selected value and which
//      function that case maps to (e.g. case 1 → autonskillsActual).
//   2. Extracts that function's body from autonomous.cpp.
//   3. Parses every movement call in the body into a JS command object.
//   4. Patches simulator.html in-place: replaces the matching auton entry
//      in AUTONS{} and marks the correct <option> as selected.
//
// WATCHED FILES (any save triggers a re-sync):
//   custom/src/user.cpp
//   custom/src/autonomous.cpp
//   custom/include/autonomous.h
//   custom/include/motor-control.h
//
// NOTE: This script edits simulator.html's JS command tables directly.
//       For live browser auto-refresh without editing HTML, use the
//       "↺ Update Code" button in the simulator with VS Code Live Server.
// ──────────────────────────────────────────────────────────────────────────────

const fs   = require('fs');
const path = require('path');

const ROOT     = __dirname;
const USER_CPP = path.join(ROOT, 'custom/src/user.cpp');
const AUTO_CPP = path.join(ROOT, 'custom/src/autonomous.cpp');
const SIM_HTML = path.join(ROOT, 'simulator.html');

const WATCH_FILES = [
  USER_CPP,
  AUTO_CPP,
  path.join(ROOT, 'custom/include/autonomous.h'),
  path.join(ROOT, 'custom/include/motor-control.h'),
];

// ──────────────────────────────────────────────────────────────────────────────
// STEP 1 — read user.cpp and find which auton is selected
//
// Looks for the pattern:
//   int auton_selected = N;
//   switch(auton_selected) {
//     case N:
//       functionName();     ← this is the function we want to parse
//
// Returns { caseNum: N, fnName: 'functionName' }, or null on failure.
// ──────────────────────────────────────────────────────────────────────────────
function findSelectedAuton(userSrc) {
  // Strip comments first so commented-out case lines don't confuse us
  const clean = userSrc
    .replace(/\/\*[\s\S]*?\*\//g, '')
    .replace(/\/\/[^\n]*/g, '');

  const valM = clean.match(/auton_selected\s*=\s*(\d+)/);
  if (!valM) { console.warn('  ⚠  Could not find auton_selected in user.cpp'); return null; }
  const n = valM[1];

  // Find: case N:   <whitespace>   functionName()
  const caseRe = new RegExp(`case\\s+${n}\\s*:[\\s\\n\\r]+(\\w+)\\s*\\(`);
  const caseM  = caseRe.exec(clean);
  if (!caseM) { console.warn(`  ⚠  Could not find case ${n} in user.cpp`); return null; }

  return { caseNum: parseInt(n), fnName: caseM[1] };
}

// ──────────────────────────────────────────────────────────────────────────────
// STEP 2 — extract and parse the selected function body from autonomous.cpp
//
// stripComments: removes comments while preserving line numbers so that
//   trailing // comments on each movement line can still be read from rawLines.
// extractBody:   brace-counts to find the exact function body, handles nesting.
// parseBody:     regex-matches each movement call into a command object.
// ──────────────────────────────────────────────────────────────────────────────
function stripComments(src) {
  // Replace block comments with equivalent newlines to preserve line count,
  // then remove line comments (the trailing text is captured separately later).
  return src
    .replace(/\/\*[\s\S]*?\*\//g, m => '\n'.repeat((m.match(/\n/g) || []).length))
    .replace(/\/\/[^\n]*/g, '');
}

function extractBody(src, name) {
  const re = new RegExp(`void\\s+${name}\\s*\\([^)]*\\)\\s*\\{`);
  const m  = re.exec(src);
  if (!m) return null;
  let depth = 1, i = m.index + m[0].length;
  while (i < src.length && depth > 0) {
    if (src[i] === '{') depth++;
    else if (src[i] === '}') depth--;
    i++;
  }
  return src.slice(m.index + m[0].length, i - 1);
}

function parseBody(rawBody) {
  const cmds      = [];
  const rawLines   = rawBody.split('\n');
  // cleanLines has comments stripped (for regex matching),
  // but rawLines is kept so we can still read trailing // comments for labels.
  const cleanLines = stripComments(rawBody).split('\n');

  for (let i = 0; i < cleanLines.length; i++) {
    const cl  = cleanLines[i].trim();
    const raw = rawLines[i] || '';
    if (!cl) continue;

    // Grab the trailing // comment from the original source line for the label
    const cmtM = raw.match(/\/\/\s*(.*)/);
    const cmt  = cmtM ? cmtM[1].trim().replace(/'/g, "\\'") : '';

    let m;

    // driveTo(dist [, timeout [, usePID [, voltage]]])
    m = cl.match(/\bdriveTo\s*\(\s*(-?[\d.]+)(?:\s*,\s*[\d.]+)?(?:\s*,\s*(?:true|false))?(?:\s*,\s*([\d.]+))?\s*\)/);
    if (m) {
      cmds.push({ t: 'drive', dist: +m[1], voltage: m[2] != null ? +m[2] : 12, cmt });
      continue;
    }

    // driveChassis(left, right)  —  skip stop calls (0, 0)
    m = cl.match(/\bdriveChassis\s*\(\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*\)/);
    if (m) {
      const l = +m[1], r = +m[2];
      if (l === 0 && r === 0) continue;
      cmds.push({ t: 'drive', dist: l < 0 ? -3 : 3, voltage: Math.abs(l),
                  cmt: cmt || 'driveChassis (dist ≈ 3in — set manually)' });
      continue;
    }

    // turnToAngle(angle [, ...])
    m = cl.match(/\bturnToAngle\s*\(\s*(-?[\d.]+)/);
    if (m) { cmds.push({ t: 'turn', angle: +m[1], cmt }); continue; }

    // moveToPoint(x, y, dir [, ...])
    m = cl.match(/\bmoveToPoint\s*\(\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)/);
    if (m) { cmds.push({ t: 'moveToPoint', x: +m[1], y: +m[2], dir: +m[3], cmt }); continue; }

    // turnToPoint(x, y, dir [, ...])
    m = cl.match(/\bturnToPoint\s*\(\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)/);
    if (m) { cmds.push({ t: 'turnToPoint', x: +m[1], y: +m[2], dir: +m[3], cmt }); continue; }

    // boomerang(x, y, dir, angle, dlead [, ...])
    m = cl.match(/\bboomerang\s*\(\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*,\s*([\d.]+)/);
    if (m) { cmds.push({ t: 'boomerang', x: +m[1], y: +m[2], dir: +m[3], angle: +m[4], dlead: +m[5], cmt }); continue; }

    // wait(ms, msec)
    m = cl.match(/\bwait\s*\(\s*([\d.]+)\s*,\s*msec\s*\)/);
    if (m) { cmds.push({ t: 'wait', ms: +m[1], cmt }); continue; }
  }

  return cmds;
}

// ──────────────────────────────────────────────────────────────────────────────
// STEP 3 — build the JS command block string that will be injected into
//          simulator.html's AUTONS{} object.
//
// Output looks like:
//   autonskillsActual: { name: 'autonskillsActual', commands: [
//     C('drive',  {dist:13, voltage:8}, 'driveTo(13, 8V)  // turn to face stack'),
//     ...
//   ]},
// ──────────────────────────────────────────────────────────────────────────────
function lbl(base, cmt) { return base + (cmt ? '  // ' + cmt : ''); }

function cmdToJS(c) {
  switch (c.t) {
    case 'drive':
      return `    C('drive',       {dist:${c.dist}, voltage:${c.voltage}},  '${lbl(`driveTo(${c.dist}, ${c.voltage}V)`, c.cmt)}'),`;
    case 'turn':
      return `    C('turn',        {angle:${c.angle}},  '${lbl(`turnToAngle(${c.angle})`, c.cmt)}'),`;
    case 'turnToPoint':
      return `    C('turnToPoint', {x:${c.x}, y:${c.y}, dir:${c.dir}},  '${lbl(`turnToPoint(${c.x}, ${c.y}, ${c.dir})`, c.cmt)}'),`;
    case 'moveToPoint':
      return `    C('moveToPoint', {x:${c.x}, y:${c.y}, dir:${c.dir}},  '${lbl(`moveToPoint(${c.x}, ${c.y}, ${c.dir})`, c.cmt)}'),`;
    case 'boomerang':
      return `    C('boomerang',   {x:${c.x}, y:${c.y}, dir:${c.dir}, angle:${c.angle}, dlead:${c.dlead}},  '${lbl(`boomerang(${c.x}, ${c.y}, ${c.dir}, ${c.angle}, ${c.dlead})`, c.cmt)}'),`;
    case 'wait':
      return `    C('wait',        {ms:${c.ms}},  '${lbl(`wait(${c.ms}ms)`, c.cmt)}'),`;
    default:
      return '';
  }
}

function buildBlock(name, cmds) {
  const body = cmds.map(cmdToJS).filter(Boolean).join('\n');
  return `  ${name}: { name: '${name}', commands: [\n${body}\n  ]},`;
}

// ──────────────────────────────────────────────────────────────────────────────
// STEP 4 — patch simulator.html in-place:
//   patchCommandBlock   replaces the existing auton entry in AUTONS{} with
//                       the freshly built block (brace-depth scan).
//   patchSelectedOption strips 'selected' from all <option> tags and adds it
//                       (with a "(case N — current)" suffix) to the right one.
// ──────────────────────────────────────────────────────────────────────────────
function patchCommandBlock(html, name, newBlock) {
  // Find the start of this auton's entry, e.g. "  autonskillsActual: {"
  const marker = `  ${name}: {`;
  const si = html.indexOf(marker);
  if (si === -1) {
    // New auton — insert before the closing }; of AUTONS
    const autonsStart = html.indexOf('const AUTONS = {');
    const autonsEnd   = html.indexOf('\n};', autonsStart);
    if (autonsStart === -1 || autonsEnd === -1) {
      console.warn(`  ⚠  Cannot locate AUTONS block in simulator.html`);
      return html;
    }
    console.log(`  inserting new '${name}' entry into AUTONS`);
    return html.slice(0, autonsEnd) + '\n\n' + newBlock + html.slice(autonsEnd);
  }
  let depth = 0, i = si, started = false;
  while (i < html.length) {
    const ch = html[i];
    if (ch === '{' || ch === '[') { depth++; started = true; }
    else if (ch === '}' || ch === ']') {
      if (--depth === 0 && started) {
        let end = i + 1;
        if (html[end] === ',') end++;
        return html.slice(0, si) + newBlock + html.slice(end);
      }
    }
    i++;
  }
  console.warn(`  ⚠  Could not find end of '${name}' block`);
  return html;
}

function patchSelectedOption(html, name, caseNum) {
  // Remove 'selected' from all options and strip any existing " (current)" suffix
  html = html.replace(/(<option\s[^>]*?)(\s+selected)([^>]*>)/g, '$1$3');
  html = html.replace(/(<option[^>]*value="[^"]*">[^<]+?)\s*\(current\)/g, '$1');

  // Add 'selected' + "(current)" to the matching option
  const re = new RegExp(`(<option[^>]*value="${name}">)([^<]+)`);
  if (re.test(html)) {
    html = html.replace(re, (_, tag, text) => {
      const cleanText = text.trim().replace(/\s*\(case \d+ [^)]*\)/, '');
      return tag.replace('value=', 'selected value=') + `${cleanText} (case ${caseNum} — current)`;
    });
  } else {
    // New auton — add option to the dropdown before </select>
    console.log(`  inserting new '${name}' option into dropdown`);
    html = html.replace(
      '</select>',
      `  <option selected value="${name}">${name} (case ${caseNum} — current)\n    </select>`
    );
  }
  return html;
}

// ──────────────────────────────────────────────────────────────────────────────
// MAIN SYNC  —  orchestrates steps 1-4 for one sync pass
// ──────────────────────────────────────────────────────────────────────────────
function doSync() {
  console.log('\n─── syncing ───────────────────────────────────────');

  let userSrc, autoCpp, html;
  try { userSrc  = fs.readFileSync(USER_CPP,  'utf8'); } catch(e) { console.error('Cannot read user.cpp:',       e.message); return; }
  try { autoCpp  = fs.readFileSync(AUTO_CPP,  'utf8'); } catch(e) { console.error('Cannot read autonomous.cpp:', e.message); return; }
  try { html     = fs.readFileSync(SIM_HTML,  'utf8'); } catch(e) { console.error('Cannot read simulator.html:', e.message); return; }

  // 1. Find selected auton
  const sel = findSelectedAuton(userSrc);
  if (!sel) { console.log('  sync aborted — could not determine active auton'); return; }
  console.log(`  active auton: case ${sel.caseNum} → ${sel.fnName}()`);

  // 2. Parse that function
  const body = extractBody(autoCpp, sel.fnName);
  if (!body) { console.log(`  ⚠  ${sel.fnName}() not found in autonomous.cpp`); return; }
  const cmds = parseBody(body);
  console.log(`  parsed ${cmds.length} movement commands`);

  // 3. Build new command block
  const block = buildBlock(sel.fnName, cmds);

  // 4. Patch HTML
  const h1 = patchCommandBlock(html, sel.fnName, block);
  const h2 = patchSelectedOption(h1, sel.fnName, sel.caseNum);

  if (h2 === html) {
    console.log('  simulator.html already up to date');
  } else {
    fs.writeFileSync(SIM_HTML, h2, 'utf8');
    console.log('  simulator.html updated ✓');
  }
}

// ──────────────────────────────────────────────────────────────────────────────
// ENTRY POINT
// ──────────────────────────────────────────────────────────────────────────────
const onceMode = process.argv.includes('--once');

console.log('simulator-sync  (cross-platform, no dependencies)');
console.log(`mode: ${onceMode ? 'run once' : 'watch'}`);

doSync();

if (!onceMode) {
  console.log('\nwatching for changes... (Ctrl+C to stop)');
  let debounce = null;
  for (const f of WATCH_FILES) {
    if (!fs.existsSync(f)) { console.warn(`  ⚠  not found, skipping watch: ${path.relative(ROOT, f)}`); continue; }
    fs.watch(f, () => {
      clearTimeout(debounce);
      debounce = setTimeout(() => {
        console.log(`\n[change detected: ${path.basename(f)}]`);
        doSync();
      }, 300);  // debounce — wait 300ms for the editor to finish writing
    });
    console.log(`  watching: ${path.relative(ROOT, f)}`);
  }
}
