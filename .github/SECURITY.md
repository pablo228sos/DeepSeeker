# Safety and security

## Vehicle safety

The firmware controls real thrusters. Test with propellers removed whenever motion is not required. Use a physical recovery line during early water trials, keep a manual stop path available, and verify the neutral ESC pulse before arming.

Autonomous missions must stay inside a visually supervised area and away from swimmers, boats, wildlife, and restricted zones. This repository does not provide collision avoidance or certified fail-safe behavior.

## Credentials

Historical firmware snapshots contain the field-test Wi-Fi password `12345678`. It is preserved for archival accuracy and must not be reused. New deployments should move credentials to an ignored local header and use a new password.

## Reporting

Use GitHub private vulnerability reporting for credential leaks or remotely exploitable control issues. Use a normal issue for reproducible non-security bugs.
