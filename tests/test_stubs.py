# Copyright (c) 2026 srounet
# SPDX-License-Identifier: MIT
"""Checks the hand-written type stubs against the compiled module.

src/wow_navmesh/__init__.pyi repeats every docstring from src/bindings.cpp (it's written
by hand to use the Point3/PolyRef aliases). This compares the two through nanobind's own
stub generator: same members, parameter names, defaults and docstrings. Needs nanobind
installed; skipped otherwise.
"""

import ast
import pathlib

import pytest

stubgen = pytest.importorskip("nanobind.stubgen")

from wow_navmesh import _wow_navmesh  # noqa: E402

STUBS = pathlib.Path(__file__).resolve().parents[1] / "src" / "wow_navmesh" / "__init__.pyi"


def _members(source):
    """Maps each "Class.member" in stub source to (kind, parameters, defaults, docstring).

    kind is "class", "def" (a method or property), "attr" (an annotated attribute, which
    the hand-written stubs document in their class's Attributes: section) or "value" (an
    enum member). Docstrings are whitespace-normalized; undocumented dunders are left out.
    """
    members = {}

    def doc(text):
        return " ".join((text or "").split())

    def walk(body, prefix):
        for node, after in zip(body, body[1:] + [None]):
            if isinstance(node, ast.ClassDef):
                members[prefix + node.name] = ("class", (), (), doc(ast.get_docstring(node)))
                walk(node.body, prefix + node.name + ".")
            elif isinstance(node, ast.FunctionDef):
                if not any(getattr(d, "attr", None) == "setter" for d in node.decorator_list):
                    members[prefix + node.name] = (
                        "def",
                        tuple(a.arg for a in node.args.args),
                        tuple(ast.unparse(d) for d in node.args.defaults),
                        doc(ast.get_docstring(node)),
                    )
            elif prefix and isinstance(node, (ast.Assign, ast.AnnAssign)):
                target = node.targets[0] if isinstance(node, ast.Assign) else node.target
                is_doc = isinstance(after, ast.Expr) and isinstance(after.value, ast.Constant)
                kind = "value" if isinstance(node, ast.Assign) else "attr"
                members[prefix + target.id] = (kind, (), (), doc(after.value.value if is_doc else ""))

    walk(ast.parse(source).body, "")
    return {k: v for k, v in members.items() if v[3] or not k.rpartition(".")[2].startswith("__")}


def test_stubs_match_the_compiled_module():
    gen = stubgen.StubGen(_wow_navmesh)
    gen.put(_wow_navmesh)
    generated = _members(gen.get())
    written = _members(STUBS.read_text(encoding="utf-8"))

    problems = []
    for name, (kind, params, defaults, doc) in generated.items():
        cls, _, attr = name.rpartition(".")
        mine = written.get(name)
        if mine is None:
            problems.append(f"{name}: missing from the stubs")
        elif mine[0] == "attr":
            if f"{attr}: {doc}" not in written[cls][3]:
                problems.append(f"{name}: {cls}'s Attributes: section should say {attr}: {doc}")
        elif kind == "class":
            if not mine[3].startswith(doc):
                problems.append(f"{name}: docstring should start with {doc!r}, not {mine[3]!r}")
        elif mine[1:] != (params, defaults, doc):
            problems.append(f"{name}: stubs have {mine[1:]}, the module {(params, defaults, doc)}")
    problems += [f"{name}: in the stubs, not the module" for name in written.keys() - generated.keys()]
    assert not problems, "\n".join(problems)
