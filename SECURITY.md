# Safety and security

## Vehicle safety

The firmware controls real thrusters. Test with propellers removed whenever motion is not required. Use a physical recovery line during early water trials, keep a manual stop path available, and verify that the neutral ESC pulse is correct before arming.

Autonomous missions must stay inside a visually supervised area and away from swimmers, boats, wildlife, and restricted zones. This repository does not provide collision avoidance or certified fail-safe behavior.

## Credentials

Historical firmware snapshots may contain field-test Wi-Fi credentials. Do not reuse them. New deployments should move credentials to an ignored local header and use a new password.

## Reporting

Open a private security report through the repository host for credential leaks or remotely exploitable control issues. Use a normal issue for reproducible non-security bugs.
