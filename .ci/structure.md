# The structure of PCL CI

## Summary

![ci-structure](./ci-structure.svg)

## Main pipeline

**Location:** `.ci/azure-pipelines/azure-pipelines.yaml`.

### Formatting

This stage will be done before both build & documentation pipeline.

### Commit Diff

Use `git diff` patterns to check code changes between HEAD & master (to determine which pipeline, build or documentation, should run next).

## Build Pipeline

**Location:** `.ci/azure-pipelines/build-pipelines.yaml`, with templates in `.ci/azure-pipelines/build` directory.

In this pipeline, *Build GCC/Clang/MSVC* stages run in parallel.

### Build GCC

Build PCL source codes in Ubuntu 18.04(oldest LTS) and Ubuntu 20.10(latest version). Then run the unit test.

If this stage completes successfully, it will trigger the documentation pipeline.

### Build Clang

Build PCL source codes in macos Catalina 10.15, macos Mojave 10.14 and Ubuntu 20.04. Then run the unit test.

### Build MSVC

Build PCL source codes in Windows Server 2019 with Visual Studio 2019 (x86 and x64 respectively). Then run the unit test.

## Documentation Pipeline

**Location:** `.ci/azure-pipelines/docs-pipelines.yaml`, with templates `documentation.yaml` for documentation generation and `tutorials.yaml` for tutorials build.

In this pipeline, *Generate Documentation* and *Commit Diff* stages run in parallel.

### Generate Documentation

### Commit Diff

Use `git diff` patterns to check two kinds of code changes: 1) PCL source codes change. 2) tutorial codes change in doc/tutorials/content/sources.

If one of them meets, then tutorial stage will be built.

### Build Tutorials

In this stage, we build and install PCL before using that to build tutorials, for the purposes of testing the tutorial source codes to make sure they work with the latest PCL codes well. 

## Reference

[Detailed discussion about the structure and stages](https://github.com/PointCloudLibrary/pcl/pull/4691)

[Some ideas about the enhancement of CI structure](https://github.com/PointCloudLibrary/pcl/pull/4737)

[Azure Pipeline: Trigger one pipeline after another](https://docs.microsoft.com/en-us/azure/devops/pipelines/process/pipeline-triggers?tabs=yaml&view=azure-devops#branch-considerations)

[Azure Pipeline: YAML schema reference](https://docs.microsoft.com/en-us/azure/devops/pipelines/yaml-schema?view=azure-devops&tabs=example%2Cparameter-schema#stage)