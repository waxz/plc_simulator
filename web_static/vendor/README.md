# vendor/

Third-party JS vendored directly into this repo (no CDN - this UI may run
on a plant-floor network with no internet access, and a build step would
be one more thing to keep working).

- **chart.umd.js** - [Chart.js](https://www.chartjs.org) v4.4.4, MIT
  licensed. Downloaded from `https://cdn.jsdelivr.net/npm/chart.js@4.4.4/dist/chart.umd.js`.
  To upgrade: download the same URL with a newer version pinned, replace
  this file, and check `web_static/app.js`'s chart setup still works
  (Chart.js's config API has had breaking changes across major versions).
