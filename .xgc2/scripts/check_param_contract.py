#!/usr/bin/env python3
"""Validate the hover thrust node parameter contract.

This keeps the public YAML/launch surface aligned with the parameters that
HoverThrustEstimatorNode::loadParams() actually reads.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
CONFIG_PATH = REPO_ROOT / "hover_thrust_estimator/config/hover_thrust_estimator.yaml"
NODE_CPP_PATH = REPO_ROOT / "hover_thrust_estimator/src/hover_thrust_estimator_node.cpp"
NODE_HEADER_PATH = REPO_ROOT / "hover_thrust_estimator/include/hover_thrust_estimator/hover_thrust_estimator_node.h"
TYPES_PATH = REPO_ROOT / "hover_thrust_estimator/include/hover_thrust_estimator/common/types.h"
LAUNCH_PATH = REPO_ROOT / "hover_thrust_estimator/launch/hover_thrust_estimator.launch"
PARAM_TEST_LAUNCH_PATH = (
    REPO_ROOT / "hover_thrust_estimator/test/hover_thrust_estimator_node_params.test"
)

DIRECT_NODE_PARAMS = {
    "imu_topic": "imu_topic_",
    "target_attitude_topic": "target_attitude_topic_",
    "altitude_topic": "altitude_topic_",
    "estimate_state_topic": "estimate_state_topic_",
    "debug_trace_topic": "debug_trace_topic_",
    "loop_rate": "loop_rate_",
}

CONFIG_FIELD_PARAM_ALIASES = {
    "publish_rate_hz": "publish_rate",
    "raw_update_rate_hz": "raw_update_rate",
}


def fail(message: str) -> None:
    print(f"parameter contract check failed: {message}", file=sys.stderr)
    raise SystemExit(1)


def top_level_yaml_values(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for lineno, line in enumerate(path.read_text().splitlines(), start=1):
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        match = re.match(r"^([A-Za-z_][A-Za-z0-9_]*)\s*:\s*(.*?)\s*$", line)
        if not match:
            fail(f"{path}:{lineno}: expected a simple top-level 'name: value' entry")
        key = match.group(1)
        value = match.group(2)
        if key in values:
            fail(f"{path}:{lineno}: duplicate key '{key}'")
        values[key] = normalize_scalar(value)
    return values


def normalize_scalar(value: str) -> str:
    stripped = value.strip()
    if (
        len(stripped) >= 2
        and stripped[0] == stripped[-1]
        and stripped[0] in {"'", '"'}
    ):
        stripped = stripped[1:-1]
    return stripped


def extract_function_body(source: str, signature: str) -> str:
    start = source.find(signature)
    if start < 0:
        fail(f"missing function signature: {signature}")
    open_brace = source.find("{", start)
    if open_brace < 0:
        fail(f"missing function body for: {signature}")

    depth = 0
    for index in range(open_brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[open_brace + 1 : index]
    fail(f"unterminated function body for: {signature}")


def load_params_from_source(path: Path) -> dict[str, str]:
    source = path.read_text()
    body = extract_function_body(source, "void HoverThrustEstimatorNode::loadParams()")
    matches = re.findall(
        r"getParamWithLog\s*\(\s*private_nh_\s*,\s*\"([^\"]+)\"\s*,\s*([^,]+?)\s*,",
        body,
        flags=re.DOTALL,
    )
    loaded: dict[str, str] = {}
    for param, target in matches:
        normalized_target = " ".join(target.split())
        if param in loaded:
            fail(f"loadParams reads parameter '{param}' more than once")
        loaded[param] = normalized_target
    if not loaded:
        fail("loadParams does not read any parameters")
    return loaded


def config_fields(path: Path) -> list[str]:
    source = path.read_text()
    match = re.search(r"struct\s+HoverThrustEstimatorConfig\s*\{(?P<body>.*?)\n\};", source, re.DOTALL)
    if not match:
        fail("missing HoverThrustEstimatorConfig definition")
    fields = re.findall(r"\b(?:bool|double|float|int|uint32_t|std::string)\s+([A-Za-z_][A-Za-z0-9_]*)\s*\{", match.group("body"))
    if not fields:
        fail("HoverThrustEstimatorConfig has no parsed fields")
    return fields


def launch_names(path: Path) -> tuple[set[str], set[str]]:
    source = path.read_text()
    args = set(re.findall(r"<arg\s+name=\"([^\"]+)\"", source))
    params = set(re.findall(r"<param\s+name=\"([^\"]+)\"", source))
    return args, params


def launch_param_values(path: Path) -> dict[str, str]:
    source = path.read_text()
    matches = re.findall(
        r"<param\s+name=\"([^\"]+)\"\s+value=\"([^\"]+)\"",
        source,
    )
    values: dict[str, str] = {}
    for name, value in matches:
        if name in values:
            fail(f"{path}: duplicate <param> entry '{name}'")
        values[name] = normalize_scalar(value)
    return values


def values_equal(left: str, right: str) -> bool:
    try:
        return float(left) == float(right)
    except ValueError:
        return left == right


def without_load_params(source: str) -> str:
    signature = "void HoverThrustEstimatorNode::loadParams()"
    start = source.find(signature)
    if start < 0:
        fail(f"missing function signature: {signature}")
    open_brace = source.find("{", start)
    if open_brace < 0:
        fail("missing loadParams body")
    depth = 0
    for index in range(open_brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[:start] + source[index + 1 :]
    fail("unterminated loadParams body")


def main() -> None:
    yaml_values = top_level_yaml_values(CONFIG_PATH)
    yaml_keys = set(yaml_values)
    loaded = load_params_from_source(NODE_CPP_PATH)
    loaded_keys = set(loaded)

    if yaml_keys != loaded_keys:
        fail(
            "YAML keys and loadParams reads differ; "
            f"missing in YAML={sorted(loaded_keys - yaml_keys)}, "
            f"missing in loadParams={sorted(yaml_keys - loaded_keys)}"
        )

    expected_targets = dict(DIRECT_NODE_PARAMS)
    for field in config_fields(TYPES_PATH):
        param = CONFIG_FIELD_PARAM_ALIASES.get(field, field)
        expected_targets[param] = f"estimator_config_.{field}"

    if loaded_keys != set(expected_targets):
        fail(
            "loadParams reads do not match node/config parameter model; "
            f"unexpected={sorted(loaded_keys - set(expected_targets))}, "
            f"missing={sorted(set(expected_targets) - loaded_keys)}"
        )

    wrong_targets = {
        param: (loaded[param], expected)
        for param, expected in expected_targets.items()
        if loaded[param] != expected
    }
    if wrong_targets:
        fail(f"loadParams targets are wrong: {wrong_targets}")

    args, launch_params = launch_names(LAUNCH_PATH)
    allowed_extra_args = {"ns", "config_file"}
    if args - yaml_keys - allowed_extra_args:
        fail(f"launch exposes unknown args: {sorted(args - yaml_keys - allowed_extra_args)}")
    if yaml_keys - args:
        fail(f"launch is missing args for YAML parameters: {sorted(yaml_keys - args)}")
    if launch_params != yaml_keys:
        fail(
            "launch <param> entries must match YAML/loadParams keys; "
            f"unexpected={sorted(launch_params - yaml_keys)}, missing={sorted(yaml_keys - launch_params)}"
        )

    test_params = launch_param_values(PARAM_TEST_LAUNCH_PATH)
    if set(test_params) != yaml_keys:
        fail(
            "node parameter non-default test must set every YAML/loadParams key; "
            f"unexpected={sorted(set(test_params) - yaml_keys)}, missing={sorted(yaml_keys - set(test_params))}"
        )
    default_values_in_test = sorted(
        key for key, value in test_params.items() if values_equal(value, yaml_values[key])
    )
    if default_values_in_test:
        fail(
            "node parameter non-default test must not reuse YAML default values: "
            f"{default_values_in_test}"
        )

    node_cpp_without_load = without_load_params(NODE_CPP_PATH.read_text())
    node_sources_for_usage = node_cpp_without_load + "\n" + NODE_HEADER_PATH.read_text()
    for param, target in DIRECT_NODE_PARAMS.items():
        if not re.search(rf"\b{re.escape(target)}\b", node_sources_for_usage):
            fail(f"direct node parameter '{param}' target '{target}' is read but not used")

    if "runtime_.setConfig(estimator_config_)" not in node_cpp_without_load:
        fail("estimator_config_ is read but not passed to HoverThrustEstimatorRuntime::setConfig")
    if "config_utils::normalizeLoopAndEstimatorRates(loop_rate_, estimator_config_)" not in NODE_CPP_PATH.read_text():
        fail("loop_rate_ and estimator_config_ are not normalized after parameter loading")

    print(
        "Parameter contract check passed: "
        f"{len(yaml_keys)} YAML keys, {len(loaded)} loadParams reads, "
        f"{len(config_fields(TYPES_PATH))} estimator config fields"
    )


if __name__ == "__main__":
    main()
