# vex-sdk

Rust bindings to VEXos system APIs.

This repository provides libraries for interacting with low-level VEXos system APIs from Rust. These crates are intended to to serve as a foundation for building higher level VEX libraries, as well as an effort to document VEXos system functions.

## Contents

- [`vex-sdk`]: FFI bindings to VEX's platform SDKs.
- [`vex-sdk-vexcode`]: A build script helper to download and link to official VEXcode SDKs.
- [`vex-sdk-jumptable`]: Open-source implementation of VEXos system APIs using firmware jump addresses.
- [`vex-sdk-pros`]: A crate that links to the PROS kernel as a provider for SDK functions, and bridges API incompatibilities between `vex-sdk` and VEX's partner SDK (`libv5rts.a`).
- [`vex-sdk-mock`]: A stubbed implementation of the VEX SDK for testing programs using the `vex-sdk` crate on non-VEXos targets.

### Disclaimer

This copy of `vex-sdk` only includes a modified version of `vex-sdk-mock`, and the other sdk implementations are provided by Cargo. This copy is only intended for use in this repository, and support for outside use is not guaranteed. An original copy of `vex-sdk` can be found [here](https://github.com/vexide/vex-sdk/tree/main).

[`vex-sdk`]: https://github.com/vexide/vex-sdk/tree/main/packages/vex-sdk
[`vex-sdk-vexcode`]: https://github.com/vexide/vex-sdk/tree/main/packages/vex-sdk-vexcode
[`vex-sdk-jumptable`]: https://github.com/vexide/vex-sdk/tree/main/packages/vex-sdk-jumptable
[`vex-sdk-pros`]: https://github.com/vexide/vex-sdk/tree/main/packages/vex-sdk-pros
[`vex-sdk-mock`]: https://github.com/vexide/vex-sdk/tree/main/packages/vex-sdk-mock
