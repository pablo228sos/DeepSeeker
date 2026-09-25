# Contributing

DeepSeeker is a field-research repository. Contributions are most useful when they are tied to a reproducible test.

## Workflow

1. Create a branch from the current `main` branch. Do not push directly to `main`.
2. Keep each branch focused on one change or one field-test dataset.
3. Open a pull request and explain what changed, why it changed, and how it was verified.
4. Wait for the required checks and review before merging.
5. Resolve review conversations and update the branch instead of replacing its history.

External contributors may work from a fork. Project collaborators may create branches directly in this repository.

## Field evidence

1. Describe the hardware revision, firmware version, location, water conditions, and test procedure.
2. Attach the relevant serial log, telemetry file, or a small representative image sample.
3. Separate measured observations from interpretation.
4. Add or update a test when changing analysis code.
5. Do not rewrite original mission archives. Derived files belong in a separate directory with a short method note.

For navigation changes, include a dry test and a tethered-water test before an unrestricted autonomous run. For imaging changes, include full-resolution comparison frames and record camera-to-bottom distance whenever possible.
