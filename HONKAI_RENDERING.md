# HonkaiRendering integration branch

Based on upstream `ruyo/VRM4U` tag `v1.2026.10.17` (`7395b50e9aa50245c8677132d805e8a8eff10093`). This fork retains the upstream license and commit ancestry.

The PMX MToon conversion preserves Assimp's diffuse color and opacity after generic MToon defaults are applied. Models can use zero opacity for hidden expression or clothing layers; turning these layers opaque obscures the face and clothing. Fractional opacity also selects a translucent blend mode. Other formats follow the upstream path.

PMX LOD0 explicitly keeps 100% of source vertices and triangles. UE 5.8's default reduction settings otherwise halve newly constructed PMX meshes before the runtime buffers are uploaded.

The bundled Assimp fork exports the PMX two-sided flag through `AI_MATKEY_TWOSIDED`, and outline settings through `$mat.mmd.edgeEnabled`, `$mat.mmd.edgeSize`, and `$mat.mmd.edgeColor`. VRM4U exposes outline settings as `mmd_EdgeEnabled`, `mmd_EdgeSize`, and `mmd_EdgeColor` material parameters. Its normal material selection preserves source culling unless the caller explicitly requests force-two-sided.

The PMX parser is unchanged. HonkaiRendering uses the existing loader and uploads its runtime PMX textures before display. Per-vertex PMX edge scale is not added by this patch.

PMX sphere maps in multiply/add modes (1/2) are exported as Assimp reflection textures with `$mat.mmd.sphereMode`. VRM4U exposes `mmd_SphereTexture` and `mmd_SphereMode`, reusing its image decoder and appending these textures after diffuse textures to preserve existing indices. Missing/undecodable sphere textures are ignored; sphere paths must stay within the PMX directory. Extra-UV mode 3 is intentionally unsupported. Geometry, skinning and PMX parsing are unchanged.

Custom PMX toon texture paths are exported through `$tex.mmd.toonPath` and decoded into `mmd_ToonTexture`. Toon and sphere images share the same path validation, decoder and deduplication cache; diffuse indices remain unchanged. Built-in shared toon indices have no file and are left to the consuming renderer's fallback. The viewer samples the authored vertical gradient; the loader does not recolor it.

`$mat.mmd.toonEnabled` / `mmd_ToonEnabled` distinguishes an authored disabled toon slot (custom index -1) from a missing custom file or built-in shared toon. A disabled slot must not receive the viewer's fallback toon shadow; sphere effects remain independent. Older imports without the property retain fallback lighting. The material property test covers both enabled and disabled states.

## Assimp source and rebuild

`ThirdParty/assimp-source` pins [the Assimp fork](https://github.com/furinasdog/assimp), based on `ruyo/assimp` commit `06c4e29a3c873648a107de9c1ac50f687b4d50d8` (6.0.2), matching the original bundled DLL revision. Its BSD license and existing dependency notices are retained. Windows x64 DLL/import-library filenames remain compatible with VRM4U's build rules.

The optional source submodule uses `update = none` so ordinary recursive project clones do not download Assimp's large upstream test assets. To rebuild, initialize it explicitly:

```powershell
git submodule update --init --checkout ThirdParty/assimp-source
```

From an x64 Visual Studio developer shell with CMake and Ninja available:

```powershell
cmake -S ThirdParty/assimp-source -B build-assimp -G Ninja -DCMAKE_BUILD_TYPE=Release -DASSIMP_BUILD_TESTS=OFF -DASSIMP_BUILD_ASSIMP_TOOLS=OFF -DASSIMP_BUILD_ZLIB=ON -DASSIMP_INSTALL=OFF -DLIBRARY_SUFFIX=-vc141-mt -DASSIMP_WARNINGS_AS_ERRORS=OFF
cmake --build build-assimp --parallel 8
```

Use a native Ninja executable. Upstream warnings remain visible; warnings-as-errors is disabled for compatibility with newer MSVC. Close Unreal before replacing `ThirdParty/assimp/bin/x64/assimp-vc141-mt.dll` and `ThirdParty/assimp/lib/x64/Release/assimp-vc141-mt.lib` with the corresponding files from `build-assimp/bin` and `build-assimp/lib`. Rebuild the UE project normally, without Live Coding/hot reload.

Validation: `VRM4U.PMX.MaterialColorAndOpacity` checks diffuse, hidden/fractional opacity, culling, and outline properties. Load actual PMX models and compare source/Assimp/UE face counts and material attributes; property tests alone do not prove import fidelity. Project smoke checks and visual results are recorded in the parent project.

## VTuber runtime morph metadata

UE 5.8 editor binaries running with `-game` have `WITH_EDITOR=1` but no `GEditor`. `USkeleton::AccumulateCurveMetaData` creates new metadata through a default-enabled undo transaction, which dereferences `GEditor`. Runtime morph conversion now calls `AddCurveMetaData(name, false)` and sets the morph flag explicitly. Existing metadata, morph registration, mesh rebuilding, and older UE version paths are retained. `VRM4U.PMX.RuntimeMorphMetadata` is a client-context Automation regression: run it with `-game -nullrhi` and verify that its runtime/GEditor-null assertions pass; an editor-context test does not verify this case. The test covers metadata creation and repeat registration, not GPU deformation.


## Runtime PMX physics and animation safety

The existing Assimp PMX reader additionally exports raw rigid bodies and joints. `UVrmPmxPhysicsData` retains the original modes, units, collision-mask bits, limits and springs. The raw mask is passed directly to Bullet; it must not be complemented. Finite damping values outside [0,1] are retained in metadata and clamped only when constructing native bodies, matching Bullet's own behavior.

`FVrmPmxPhysics` is an optional Windows adapter around the host's UMTNativePlugin fork and pinned Bullet source. It consumes a full mesh-indexed component-space animated pose and returns the complete simulated pose, including descendants and mode-2 local-position handling under a physical parent. It does not convert PMX joints into VRM0 spring chains. It uses fixed60Hz steps, bounded catch-up, long-frame reset, scale checks, and finite/index validation. Unsupported joint types fail explicitly.

The current host fork uses Bullet 3.07 Spring2, with authored angular stiffness rather than the former motor's length-scale-squared conversion. Both joint damping coefficients are `.5`; zero-stiffness axes retain velocity damping. Directly linked pairs are excluded from contact while all other PMX masks remain active. The original physical metadata is unchanged. See the host `PHYSICS.md` for dependency preparation, measured improvements and remaining strict-residual limits.

The host prepares `Tools/UMTNativePlugin/build-static-ue144/bundled/UMTNativePlugin.lib` before building. VRM4U detects that optional archive relative to the host project; builds without it retain the importer and return an explicit unavailable-runtime error. Runtime data does not store pointers into the temporary Assimp scene. Use matching rebuilt Assimp headers, DLL and import library together after the metadata ABI extension.

The VRM spring path also validates raw-to-compact bone indices. Runtime PMX vertex morph registration works without opening an editor transaction through a null GEditor. Host animation proxies must register the inherited VRM4U proxy pointer and initialize the entire reference pose before applying targets.

Tests under `VRM4U.PMX` cover original material properties, physical metadata persistence, coordinate basis, gravity, collision groups, mode-2 parent chains and finite damping compatibility. Real-model image and motion evidence belongs to the host project; those unit checks do not establish collision-free appearance or official-game fidelity.
