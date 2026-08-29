/**
 * ActionPhysics build.
 *
 *   node build.js              concatenate to build/actionphysics.js
 *   node build.js --minify     also emit build/actionphysics.min.js
 *
 * Fully synchronous: when this process exits, the output file is written. A test run
 * immediately afterwards reads the build it just produced, never the previous one.
 * (An async build silently serving stale output to a test sweep produces confidently wrong results.)
 *
 * Zero dependencies for the shipped library; the build itself may use whatever it likes.
 */
'use strict';

const fs = require('fs');
const path = require('path');

const NL = String.fromCharCode(10);
const ROOT = __dirname;
const SRC = path.join(ROOT, 'src');
const OUT_DIR = path.join(ROOT, 'build');
const manifest = require('./sources.js');

// Allowed Math.* calls in the simulation path. Everything else is implementation-defined
// across JS engines (the spec only requires correct rounding for +,-,*,/,% and sqrt), so
// it would break cross-machine determinism. Transcendentals live in src/math/Scalar.js.
// PI/E and friends are exact IEEE-754 constants, not implementation-approximated functions.
const MATH_ALLOWLIST = new Set([
    'sqrt', 'abs', 'min', 'max', 'floor', 'ceil', 'round', 'sign', 'trunc',
    'PI', 'E', 'LN2', 'LN10', 'LOG2E', 'LOG10E', 'SQRT2', 'SQRT1_2'
]);

function fail(msg) {
    console.error('BUILD FAILED: ' + msg);
    process.exit(1);
}

// Every .js under src/, relative to the project root, in posix form.
function walk(dir, found) {
    for (const entry of fs.readdirSync(dir, { withFileTypes: true })) {
        const full = path.join(dir, entry.name);
        if (entry.isDirectory()) {
            walk(full, found);
        } else if (entry.name.endsWith('.js')) {
            found.push(path.relative(ROOT, full).split(path.sep).join('/'));
        }
    }
    return found;
}

// A source file present on disk but absent from the manifest would simply never appear in
// the build, surfacing later as an undefined constructor. Caught here instead.
function checkManifestComplete(onDisk) {
    const listed = new Set(manifest);
    const missing = onDisk.filter(f => !listed.has(f));
    if (missing.length) {
        fail('these files exist under src/ but are not in sources.js:\n  ' + missing.join('\n  '));
    }
    const absent = manifest.filter(f => !fs.existsSync(path.join(ROOT, f)));
    if (absent.length) {
        fail('sources.js lists files that do not exist:\n  ' + absent.join('\n  '));
    }
}

// Determinism gate - see MATH_ALLOWLIST above.
//
// Comments are blanked first (preserving line numbers) so prose explaining why a function is avoided
// does not trip the check that avoids it.
function stripComments(source) {
    var NL = String.fromCharCode(10);
    return source
        .replace(/\/\*[\s\S]*?\*\//g, function (m) {
            return m.split(NL).map(function (line) { return line.replace(/./g, " "); }).join(NL);
        })
        .replace(/(^|[^:])\/\/.*/g, function (m, p1) {
            return p1 + m.slice(p1.length).replace(/./g, " ");
        });
}

function checkDeterminism(file, source) {
    source = stripComments(source);
    const offenders = [];
    const re = /\bMath\.([a-zA-Z0-9_]+)/g;
    let m;
    while ((m = re.exec(source)) !== null) {
        if (!MATH_ALLOWLIST.has(m[1])) {
            const line = source.slice(0, m.index).split('\n').length;
            offenders.push(`${file}:${line}  Math.${m[1]}`);
        }
    }
    return offenders;
}

// ActionMath files declare a bare `class X {}`, which cannot be conditionally skipped - a class
// declaration is not an expression. Each is therefore wrapped in a block that only runs when the host
// has not already supplied that class. When it has, the bundled definition never executes and
// ActionPhysics binds the host's class instead, leaving exactly one of each in the page.
const MATH_CLASS = /^src\/math\/([A-Za-z0-9_]+)\.js$/;

// Indents a block, leaving empty lines empty rather than filling them with spaces.
function indent(text) {
    // Splits on either line ending. The sources are CRLF, and splitting on LF alone leaves a stray
    // CR at the end of every line.
    var NEWLINE = new RegExp(String.fromCharCode(13) + "?" + String.fromCharCode(10));
    return text.split(NEWLINE).map(function (line) { return line ? "    " + line : line; }).join(NL);
}

function wrapIfMath(rel, source) {
    const m = rel.match(MATH_CLASS);
    if (!m) return source;
    const cls = m[1];
    // The bundled copy defines `class X {}` at block scope, invisible to the other math files - and they
    // reference each other (Matrix4.transformNormal returns a Vector3). Rewriting the declaration to a
    // plain assignment of a class EXPRESSION puts the binding in the shared scope instead, so the files
    // see each other exactly as they do when the host supplies them.
    const named = new RegExp('^class ' + cls + '(?![A-Za-z0-9_])', 'm');
    const asExpression = source.replace(named, cls + ' = class ' + cls);
    return (
        'var ' + cls + ';' + NL +
        'if (host.' + cls + ') {' + NL +
        '    ' + cls + ' = host.' + cls + ';' + NL +
        '} else {' + NL +
        indent(asExpression) + NL +
        '}' + NL +
        'ActionPhysics.' + cls + ' = ' + cls + ';' + NL
    );
}

function build() {
    const onDisk = walk(SRC, []);
    checkManifestComplete(onDisk);

    const parts = [];
    let determinismOffenders = [];

    for (const rel of manifest) {
        const source = fs.readFileSync(path.join(ROOT, rel), 'utf8');
        determinismOffenders = determinismOffenders.concat(checkDeterminism(rel, source));
        parts.push('// ==== ' + rel + ' ====\n' + wrapIfMath(rel, source));
    }

    if (determinismOffenders.length) {
        fail(
            'non-deterministic Math.* calls (implementation-defined across JS engines).\n' +
            'Use ActionPhysics.Scalar instead.\n  ' + determinismOffenders.join('\n  ')
        );
    }

    const pkg = JSON.parse(fs.readFileSync(path.join(ROOT, 'package.json'), 'utf8'));
    const stamp = `// ActionPhysics ${pkg.version} — built ${new Date().toISOString()}\n`;
    const out = stamp + parts.join('\n\n');

    if (!fs.existsSync(OUT_DIR)) fs.mkdirSync(OUT_DIR, { recursive: true });
    const outFile = path.join(OUT_DIR, 'actionphysics.js');
    fs.writeFileSync(outFile, out);

    const lines = out.split('\n').length;
    console.log(`built ${path.relative(ROOT, outFile)}  ${manifest.length} files, ${lines} lines`);

    if (process.argv.includes('--minify')) {
        let terser;
        try {
            terser = require('terser');
        } catch (e) {
            fail('--minify needs terser: npm install --save-dev terser');
        }
        const result = terser.minify_sync
            ? terser.minify_sync(out)
            : (() => { fail('installed terser has no sync API; use terser >= 5'); })();
        if (result.error) fail('minify: ' + result.error);
        const minFile = path.join(OUT_DIR, 'actionphysics.min.js');
        fs.writeFileSync(minFile, stamp + result.code);
        console.log(`built ${path.relative(ROOT, minFile)}  ${(result.code.length / 1024).toFixed(1)} KB`);
    }
}

build();
