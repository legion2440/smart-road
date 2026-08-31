# Assets

Runtime visual assets used by the SDL2 renderer.

The smart-road atlas is stored as verified Base64 text chunks and reconstructed in memory at startup. This avoids binary transport corruption while preserving the original PNG bytes exactly.

The atlas contains:

- autonomous EV sedan;
- cyber sport GT;
- robo-taxi urban pod;
- smart electric transit bus;
- smart police interceptor;
- smart cyber ambulance;
- smart rescue fire engine;
- horizontal and vertical smart-road textures;
- four-way intersection texture;
- roadside neon-tree sprite.

The visual vehicle type does not change collision geometry or the smart-intersection safety model. Traffic-light assets and LiDAR scan rings are not used.
