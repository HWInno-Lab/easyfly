# Natural Earth land outlines

The bundled `ne_110m_land.geojson` is Natural Earth 5.1.2, 1:110 million land polygons.
Original URL and SHA-256 are in `provenance.json`. No political boundaries or
street/terrain data are included. Mercator rendering clamps polar land outlines
to its representable latitude; actual polar GPS positions are explicitly refused.

Natural Earth data are public domain according to the project's terms:
https://www.naturalearthdata.com/about/terms-of-use/

Made with Natural Earth. This coarse dataset provides global context and cannot
establish metre-scale flight geometry. Imported raster MBTiles must carry their
own name, attribution, format and zoom bounds; EasyFly performs no network downloads.
