# Handoff - Blynk legacy server work (state as of 2026-10-07)

Read MANIFEST.md first for file list, hashes and rebuild steps. This file is the context a new session needs.

## Goal
Keep a legacy Blynk server (Java, 0.41.17) running on an Orange Pi with a small SSD, and make it **recover by itself after a reboot** when a profile file is corrupted (it had one corruption about every 1-2 years even without power loss; SSD instead of SD card helped but did not cure it).

## Done
- Official blynkkk/blynk-server was deleted by Blynk in Sept 2022. Sources: Peterkn2001/blynk-server (full source, commit 39c6b17, 2022-06-15), binaries: gablau/blynk-server-binaries (0.41.17 jar, sha1 files match but provenance is unverified).
- Built the source offline (JDK 17, Maven 3.9.9). Jar entries and resources match the archive jar; class files are not byte-identical (compiler differences). Full bytecode comparison of all classes was NOT finished (system ran out of memory).
- Legacy Android app: only an APK exists (apk/Blynk.legacy1.0.0.apk, community clone "same as legacy v2.27.20"). The clone repo never contained buildable source. Not inspected or run. No iOS option.
- Found why corruption was not self-healing: profile files (`<email>.<app>.user`, JSON) were rewritten in place with no temp file/fsync, and startup restore from `backup/` (daily copies) only triggered for two error messages ("end-of-input", "Illegal character"); every other parse error silently dropped the user.
- Patch (source/selfheal.patch, working tree source/blynk-server-selfheal, branch `selfheal`, committed as dc245a4):
  - atomic write: temp file + fsync + atomic rename (JsonParser.writeUser)
  - SHA-256 trailer line `//sha256:<hex>` after the JSON in every profile and backup; mismatch = corruption; old files without trailer still load; old servers can still read new files
  - startup: any unreadable profile is restored from the newest *usable* backup, damaged copy kept in `<dataFolder>/broken/`, leftover *.tmp removed
- Result: `server/server-0.41.17-selfheal.jar` (hash in MANIFEST.md). Tested with simulated corruption (truncated, zero-filled, garbage, wrong structure, empty file, broken newest backup, one-character value change): patched loads all that have a usable backup; original 0.41.17 loaded 3 of 7.

## Decisions by the user
- Daily backups are enough; no hourly backups.
- Leave the Android app source question; do not pursue it.

## Orange Pi (found 2026-10-07)
- SSH access and account names: see PI-ACCESS.local.md in this folder (local only, not committed). Debian 9 armhf, 492 MB RAM, root on SSD /dev/sda1, /tmp is tmpfs (RAM) - do not stage files there.
- Live server: `java -jar /root/server-0.41.12-java8.jar -dataFolder /home/pi/Blynk &` in /etc/rc.local (system Java 8). Logs in /root/logs (cwd). One active user; three old users only have backups from 2022.
- 0.41.12 contains the Log4Shell-vulnerable JndiLookup class. 0.41.17 source cannot be built for Java 8 (uses `var` etc.), so Temurin 11 JRE was installed side by side: /opt/jre11 -> /opt/jdk-11.0.32.1+1-jre (sha256 checked). System Java 8 untouched.
- March 2026 incident in /root/logs/blynk.log: startup failed with "Duplicate key" (several .user files containing the active user's profile); fixed then by deleting the extra .user files by hand. The patch now handles this (see MANIFEST).
- Rehearsal done in /root/blynktest (copy of data, ports 18080/19443/18440, script run.sh): starts on Java 11, loads the real profile, saves with trailer, restores a truncated profile. Can be deleted after deployment. cmp.sh there compares graph history (HTTP /{token}/data/{pin} CSV) between live (8080) and test (18080): new server on the copied data returns the same rows as live (V2/V5/V40/V98, 16-24k rows each, none differ; live only has the newer points). Rollback checked too: 0.41.12 on Java 8 reads profile and history written by the new server (V33 value saved by new server read back, history identical).

## DEPLOYED 2026-10-07 14:04
- Live now: `/opt/jre11/bin/java -jar /root/server-0.41.17-selfheal.jar -dataFolder /home/pi/Blynk &` (rc.local line 16, cwd /, logs in /logs). Hardware rejoined 1 s after start, Android app joined; profile saved with sha256 trailer; RSS ~107 MB.
- Backups: /home/pi/Blynk.bak-2026-10-07 (taken while old server ran), /home/pi/Blynk.bak-2026-10-07-stopped (consistent, after stop), /etc/rc.local.bak-2026-10-07, /logs.bak-2026-10-07. Old jar still at /root/server-0.41.12-java8.jar.
- Rollback: `pkill -f server-0.41.17-selfheal.jar; cp /etc/rc.local.bak-2026-10-07 /etc/rc.local; cd / && setsid nohup java -jar /root/server-0.41.12-java8.jar -dataFolder /home/pi/Blynk >/dev/null 2>&1 &` (0.41.12 reads files written by the new server - tested).
- Lesson: deploy.sh used sed with `&` in the replacement and mangled rc.local line 16 for ~7 minutes; fixed and checked with diff + `bash -n`.

## LOG4J FIXED AND DEPLOYED 2026-10-07 14:27
- Commit 3b460c0: log4j 2.17.1 + Image NPE fix. Same jar name /root/server-0.41.17-selfheal.jar (SHA-256 eab02ff9...), rc.local unchanged. Previous (log4j 2.14.1) jar kept as /root/server-0.41.17-selfheal-log4j2141.jar.bak. Rehearsed on test ports first (logs, profile, history, save, restore OK); after swap hardware rejoined in 3 s.

## (resolved) SECURITY GAP FOUND AFTER FIRST DEPLOY
- The first deployed selfheal jar had log4j 2.14.1 (Log4Shell-vulnerable). Peterkn2001 source stops at 2021-05-30; the official v0.41.17 release (2021-12-14, d7e0b6c2) = same code + log4j 2.15.0, then 087b7588 log4j 2.16.0 and 8faff068 one-line NPE fix in Image widget. The archive server-0.41.17.jar (Build-Number 0.41.18-SNAPSHOT, built by doom369) has the fixed log4j. Official history to 2022-06 is in github.com/gablau/blynk-server (later commits remove local backups - do NOT take those).
- Fix: set log4j2.version to 2.17.1 (+ optional Image NPE fix), rebuild, redeploy. Old 0.41.12 was vulnerable too, so not a regression.

## Not done / next
- Reboot tested 2026-10-07 14:13: rc.local started the new server, hardware and app rejoined within seconds, user confirmed the app works.
- Cleaned up: /root/blynktest, /root/deploy.sh, a stray file from the key setup. Backups and old jar kept on purpose.
- Not covered: shutdown-time save of a just-restored profile (if the server is stopped within ~1 min of a restore, it simply restores again from the same backup next start).
- Reporting/graph data files were not examined for crash safety.
- Optional: finish the full class-by-class bytecode comparison against the archive jar (do it in small batches).

## Practical notes
- Windows: run `git config core.longpaths true` in the source folder, otherwise checkout fails with "Filename too long".
- Offline rebuild: `mvn -o -DskipTests -Dmaven.repo.local=<this folder>\libs\m2 clean package` (copy libs\m2 elsewhere first so the backup stays unchanged). tools/ has Maven 3.9.9.
- The QRGen 2.2.0 dependency is installed under the lowercase group com.github.kenglxn.qrgen in libs/m2 because jitpack only serves the capitalised QRGen group now.
- Test harness used (not saved here): a small Java class that writes users with JsonParser.writeUser, corrupts files, then calls `new FileManager(dir,"localhost").deserializeUsers()`.
