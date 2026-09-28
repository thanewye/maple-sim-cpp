# Publishing the vendordep

The Gradle project follows WPILib's 2026 native vendor template. A public release consists of immutable Maven artifacts plus a vendordep JSON whose coordinates, platform list and URLs match those artifacts.

## Before the first release

1. Choose the permanent Maven group and hosting URLs. The defaults in `publish.gradle` use `example.invalid` so an unpublished build cannot masquerade as an installable release.
2. Keep the UUID in `MapleSimCpp.json` stable forever. Change the version for releases; never mint a new UUID for a routine update.
3. Host both the Maven repository and `MapleSimCpp.json` over anonymous HTTPS. GitHub Pages is suitable because a Maven repository is a static directory tree. GitHub Packages is a poor public vendordep host when anonymous downloads require authentication.
4. Keep `frcYear`, `wpilibVersion`, NativeUtils and CI images on the same WPILib season. Publish a separate version when moving to a new season.
5. Do not overwrite an existing version. Maven artifacts and versioned vendordep JSON files should be immutable.

## Build a release candidate

Pick a semantic version and final public URLs:

```bash
./gradlew clean build publish -PreleaseMode \
  -PpublishVersion=0.1.0 \
  -PpublishGroup=com.example.maplesim \
  -PvendordepMavenUrl=https://example.github.io/maple-sim-cpp/maven \
  -PvendordepJsonUrl=https://example.github.io/maple-sim-cpp/MapleSimCpp.json
```

`build/repos/releases` is the Maven repository to upload. The expanded vendordep is `build/vendordep/MapleSimCpp.json`. The platform, header and source archives are also collected under `build/allOutputs`.

The `cppDependencies` entry must continue to match the publication exactly:

- `groupId`, `artifactId` and `version` identify the Maven artifact.
- `libName` matches the Gradle native component and produced library name.
- `headerClassifier` points to the header ZIP.
- `sharedLibrary` stays `false` while the project publishes static archives.
- Every advertised `binaryPlatforms` classifier must actually be uploaded. Remove unsupported platforms rather than publishing a dangling declaration.
- Debug archives use WPILib's normal debug classifier and naming conventions.

## Validate before release

1. Build each advertised platform, including desktop hosts and `linuxathena`. The supplied CI workflow covers Linux, roboRIO, ARM, Windows and macOS, then combines their outputs into one `Maven` artifact with WPILib's build tools.
2. Run WPILib's official vendor JSON checker against the candidate JSON and local Maven tree before upload:

   ```bash
   python3 check.py --local-maven build/repos/releases --year 2026 build/vendordep/MapleSimCpp.json
   ```

3. Upload to a staging path and run the checker again against the hosted URLs.
4. Install the hosted JSON into a fresh 2026 C++ robot project. Compile both a desktop simulation target and the roboRIO target. This catches missing headers, link-order errors and incomplete classifiers that the JSON checker cannot prove.
5. Inspect the header and binary ZIPs for the license files. The Team 5516 Iron Maple and Box2D MIT notices must accompany distributions of their code.

## Publish and announce

1. Upload `build/repos/releases` without deleting older versions.
2. Upload the generated `MapleSimCpp.json` at its stable `jsonUrl`. Keeping versioned JSON snapshots alongside it makes rollback and auditing easier.
3. Tag the exact source revision used for the artifacts and attach the generated JSON plus checksums to the release.
4. Document the supported WPILib year, platforms, known parity gaps and installation URL in the release notes.
5. Optionally submit the versioned JSON and metadata to [wpilibsuite/vendor-json-repo](https://github.com/wpilibsuite/vendor-json-repo). Inclusion in WPILib's dependency manager is reviewed case by case; it is separate from hosting a directly installable vendordep URL.

The authoritative references are WPILib's [2026 vendor template](https://github.com/wpilibsuite/vendor-template/tree/2026), [vendor JSON repository](https://github.com/wpilibsuite/vendor-json-repo), and [third-party library installation documentation](https://docs.wpilib.org/en/latest/docs/software/vscode-overview/3rd-party-libraries.html).
