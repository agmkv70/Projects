# Blynk legacy backup  (created 2026-10-07)

| Path | What | Source |
|---|---|---|
| server/server-0.41.17.jar (+.sha1) | Legacy server 0.41.17 (Log4j2 fix) - **use this one** | github.com/gablau/blynk-server-binaries, 0.41.17/ |
| server/server-0.41.17-SNAPSHOT.selfbuilt.jar | Jar built locally from source/ with JDK 17 + Maven 3.9.9 | built here; classes match the archive's SNAPSHOT jar in entries and resources, not byte-for-byte |
| source/blynk-server | Full source at commit 39c6b17d8dfb476629088c958afcdaf553b09334 (0.41.17-SNAPSHOT, 2022-06-15), shallow git clone | github.com/Peterkn2001/blynk-server |
| libs/m2 | Maven repository with every dependency needed to rebuild offline | Maven Central / jitpack |
| libs/javase-2.2.0.* , stub.pom | QRGen 2.2.0, installed under lowercase group com.github.kenglxn.qrgen (jitpack only serves it as com.github.kenglxn.QRGen) | jitpack.io |
| tools/apache-maven-3.9.9-bin.zip | Maven (SHA-512 verified against Apache) | archive.apache.org |
| apk/Blynk.legacy1.0.0.apk | Community clone of legacy Android app, "same as legacy v2.27.20", released 2021-12-22 | github.com/BlynkMobile/Blynk-Android-App release v1.0.0 |

SHA-256
- server-0.41.17.jar: 8D32029E75F7E1986AECBBA5D8CE6D800FD6A6D92F0F5C9AED05498F7A21806F
- server-0.41.17-SNAPSHOT.selfbuilt.jar: E85C290B1674C508E3E8BB6CF4471862E8E31A62BE5AE38CA30A6A5128405389
- Blynk.legacy1.0.0.apk: 850667EAE0F7B8173BDC60B45CB67125C5C2BF6A621E77DAD7A607657ECBDE9D
- apache-maven-3.9.9-bin.zip: 4EC3F26FB1A692473AEA0235C300BD20F0F9FE741947C82C1234CEFD76AC3A3C

## Rebuild offline
JDK 11+ required (built with 17). On Windows run `git config core.longpaths true` in source/blynk-server first.
    mvn -o -DskipTests -Dmaven.repo.local=<this>\libs\m2 clean package
Output: server/launcher/target/server-0.41.17-SNAPSHOT.jar

## Run
    java -jar server-0.41.17.jar -dataFolder <dir>

## Caveats
- Provenance of the archive jar and the Peterkn2001 source is unverified; Blynk's official repo is gone.
- The APK is a third-party clone, unsigned-by-Blynk; not inspected or run.
- Original-jar vs self-built comparison was not finished for all classes.

## Self-heal patch (added 2026-10-07)
- source/blynk-server-selfheal (branch selfheal, commits dc245a4 + 3b460c0) and source/selfheal.patch: changes to JsonParser.writeUser and FileManager.
- server/server-0.41.17-selfheal.jar  SHA-256 EAB02FF9C0C5030C453B64F59ACD6D45BA9E27C015E20F18CA58BF119CC8C673
- Writes of .user files and daily backups are now temp file + fsync + atomic rename (no half-written files after a crash).
- Startup: any unreadable .user file is restored from the newest *usable* backup (older ones are tried if the newest is broken); the broken file is copied to <dataFolder>/broken/ first; leftover *.tmp files are deleted.
- Tested with truncated, zero-filled, garbage, wrong-structure, empty-file and broken-newest-backup cases (original jar loaded 3 of 7 test users, patched jar 6 of 7; the 7th has no backup at all).
- Not covered: no backup exists, or all backups are broken (user is skipped and logged as an error), and up to a day of changes since the last daily backup are lost on restore.

- Checksum (added later the same day): every .user profile and backup is written as <json>, newline, `//sha256:<hex of the json>`. On load a mismatch is treated like any other corruption (restore from newest usable backup). Files without a trailer (older versions) still load; they get a trailer the next time they are saved. Older servers can read the new files (the JSON reader stops after the first value).
- Tested: single-character change inside a value (still valid JSON) is detected and restored; legacy file without trailer loads.

- Duplicate / foreign profile (added later the same day): a .user file whose content belongs to another user (name does not match `<email>.<app>.user`) is treated as corrupt and restored from its own backups; if two files still map to the same user, the newer one (lastModifiedTs) is used instead of failing the whole load. The original 0.41.12/0.41.17 servers abort with "Duplicate key" and load nobody (this happened on the Pi in March 2026).
- Tested on the Orange Pi (Java 11, copy of the real data, test ports): loads the 0.41.12 profile, saves with checksum trailer, restores a truncated profile from backup.
- Note: profiles are now created with permissions 600 (temp file + rename) instead of 644; the server runs as root, so this does not matter there.
- log4j (added later the same day, commit 3b460c0): log4j updated 2.14.1 -> 2.17.1 (Log4Shell; the Peterkn2001 source predates the official 0.41.17 release, which only bumped log4j) plus upstream one-line NPE fix in Image widget. log4j 2.17.1 jars added to libs/m2; offline rebuild verified.
