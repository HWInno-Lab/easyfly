# DBoW2 / DUtils license text recovery

The pinned ORB-SLAM3 snapshot's README explicitly identifies the ORB-SLAM2
DBoW2/DUtils distribution and references its missing LICENSE.txt. This file is
restored verbatim from official ORB-SLAM2 commit
`f2e6f51cdc8d067655d90a78c06261378e07e8f3`, path `Thirdparty/DBoW2/LICENSE.txt`:
https://github.com/raulmur/ORB_SLAM2/blob/f2e6f51cdc8d067655d90a78c06261378e07e8f3/Thirdparty/DBoW2/LICENSE.txt

The README and 12 source files match that distribution byte for byte. The two
remaining headers differ only by Boost serialization includes and methods added
by ORB-SLAM3; their existing original authorship notices are unchanged. Those
additions remain subject to ORB-SLAM3's own license. This restores the upstream
BSD-3-Clause text; it does not replace ORB-SLAM3's GPL or any per-file notice.

Retrieved 2026-10-28. LICENSE.txt SHA-256:
`ad22a7fc36c72bd4016bb3ae49437d0433eaa74e3c4768cadebcd0f3236e5aae`.
The unrelated current dorian3d/DBoW2 license was not substituted for this version.
