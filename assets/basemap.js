// Basemaps shared by the web app (served as /basemap.js) and the static
// plan / explore viewers (inlined by src/output.rs).
//
// Street: OpenFreeMap's keyless vector "positron" style. (CARTO's raster
// tiles, used before, started serving an "API KEY REQUIRED" placeholder.)
// It is fetched and merged in under everything else after the map loads,
// so if that server is unreachable the map still works, on a blank
// background, instead of failing to load at all.
// Terrain: an opaque OpenTopoMap raster layer (`bm-topo`) drawn above the
// street layers when the "terrain" radio is selected.

const STREET_STYLE_URL = 'https://tiles.openfreemap.org/styles/positron';

const BASEMAP_STYLE = {
  version: 8,
  sources: {
    topo: {
      type: 'raster',
      tiles: ['https://a.tile.opentopomap.org/{z}/{x}/{y}.png',
              'https://b.tile.opentopomap.org/{z}/{x}/{y}.png',
              'https://c.tile.opentopomap.org/{z}/{x}/{y}.png'],
      tileSize: 256, maxzoom: 17,
      attribution: 'Map data &copy; OpenStreetMap contributors, SRTM | &copy; OpenTopoMap (CC-BY-SA)'
    }
  },
  layers: [
    { id: 'bm-topo', type: 'raster', source: 'topo', layout: { visibility: 'none' } }
  ]
};

// Hook the street/terrain radios (`input[name="bm"]`) to `map` and merge the
// street style in once the map has loaded.
function setupBasemaps(map) {
  const showSelected = () => {
    const terrain = document.querySelector('input[name="bm"]:checked')?.value === 'topo';
    try {
      map.setLayoutProperty('bm-topo', 'visibility', terrain ? 'visible' : 'none');
    } catch (_) { /* style still loading: synced on 'load' */ }
  };
  document.querySelectorAll('input[name="bm"]').forEach(r => r.addEventListener('change', showSelected));
  map.on('load', async () => {
    showSelected();
    try {
      const style = await fetch(STREET_STYLE_URL).then(r => r.json());
      map.setGlyphs(style.glyphs);
      map.setSprite(style.sprite);
      for (const [id, source] of Object.entries(style.sources)) {
        if (!map.getSource(id)) map.addSource(id, source);
      }
      for (const layer of style.layers) {
        if (!map.getLayer(layer.id)) map.addLayer(layer, 'bm-topo');
      }
    } catch (e) {
      console.warn('street basemap unavailable:', e);
    }
  });
}
