// Minimal static file server for opening tests/suite.html in a browser.
const http = require('http'), fs = require('fs'), p = require('path');
const ROOT = p.join(__dirname, '..');
const TYPES = { '.html': 'text/html', '.js': 'text/javascript', '.css': 'text/css', '.json': 'application/json' };
http.createServer((req, res) => {
    let f = p.join(ROOT, decodeURIComponent(req.url.split('?')[0]));
    if (f.endsWith(p.sep) || f.endsWith('/')) f = p.join(f, 'index.html');
    fs.readFile(f, (err, data) => {
        if (err) { res.writeHead(404); return res.end('404'); }
        res.writeHead(200, { 'Content-Type': TYPES[p.extname(f)] || 'application/octet-stream' });
        res.end(data);
    });
}).listen(8777, () => console.log('serving ' + ROOT + ' on http://localhost:8777'));
