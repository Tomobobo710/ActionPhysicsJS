/**
 * Ordered source manifest.
 *
 * Explicit and ordered — no globs. Dependency order is a design decision, so it is stated
 * here rather than inferred from directory traversal - glob ordering is not a dependency graph.
 *
 * build.js fails if any .js under src/ is missing from this list, so a new file that is
 * never concatenated is caught at build time instead of as a confusing runtime error.
 */
module.exports = [
    'src/intro.js',

    // ActionMath - pasted verbatim from the shared library, never edited locally.
    // Scalar first: the others call it.
    'src/math/Scalar.js',
    'src/math/Vector2.js',
    'src/math/Vector3.js',
    'src/math/Matrix3.js',
    'src/math/Matrix4.js',
    'src/math/Quaternion.js',
    'src/math/Transform.js',

    'src/outro.js'
];
