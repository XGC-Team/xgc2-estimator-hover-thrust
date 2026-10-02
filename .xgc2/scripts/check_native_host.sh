#!/usr/bin/env bash
set -euo pipefail
# Domain-owned regression against an explicitly supplied generic Host source.
# No estimator source fallback and no writes to the runtime workspace.
: "${XGC_RUNTIME_SOURCE_ROOT:?generic Host source root required}"
: "${HTE_NATIVE_LIBRARY:?built/installed domain native ELF required}"
: "${HTE_REFERENCE_BIN:?same-core reference executable required}"
: "${HTE_HOST_WORK_DIR:?private test workspace required}"
root="$(cd "$(dirname "$0")/../.." && pwd)"
mkdir -p "$HTE_HOST_WORK_DIR"
python3 - "$root" "$XGC_RUNTIME_SOURCE_ROOT" "$HTE_HOST_WORK_DIR" <<'PY'
import pathlib,sys
owner, runtime, out = map(pathlib.Path, sys.argv[1:])
text='[package]\nname="hte-native-host-regression"\nversion="0.0.0"\nedition="2021"\n\n[[test]]\nname="hte_host"\npath='+repr(str(owner/'hover_thrust_estimator/native/host_equivalence.rs'))+'\n\n[dependencies]\n'
for name in ['xgc-rt-audit','xgc-rt-core','xgc-rt-host','xgc-rt-transport-loopback']:
 text += name+' = { path = '+repr(str(runtime/'crates'/name))+' }\n'
(out/'Cargo.toml').write_text(text)
PY
export CARGO_BUILD_JOBS=1
cargo test --offline --manifest-path "$HTE_HOST_WORK_DIR/Cargo.toml" --test hte_host -- --nocapture --test-threads=1
