"""Policy registry that permits extensions without editing campaign code."""

from __future__ import annotations

from collections.abc import Callable, Iterator

from .base import Policy, PolicyDeclaration


PolicyFactory = Callable[..., Policy]


class PolicyRegistry:
    """Name-to-factory registry with mandatory scientific declarations."""

    def __init__(self) -> None:
        self._entries: dict[str, tuple[PolicyFactory, PolicyDeclaration]] = {}

    def register(
        self,
        declaration: PolicyDeclaration,
        factory: PolicyFactory,
    ) -> None:
        if not declaration.name or declaration.name.strip() != declaration.name:
            raise ValueError("policy name must be non-empty and canonical")
        if declaration.name in self._entries:
            raise ValueError(f"policy already registered: {declaration.name}")
        self._entries[declaration.name] = (factory, declaration)

    def create(self, name: str, **kwargs) -> Policy:
        try:
            factory, _ = self._entries[name]
        except KeyError as exc:
            raise KeyError(f"unknown policy {name!r}; available={tuple(self)}") from exc
        policy = factory(**kwargs)
        if getattr(policy, "name", None) != name:
            raise ValueError(
                f"factory for {name!r} produced policy named "
                f"{getattr(policy, 'name', None)!r}"
            )
        return policy

    def declaration(self, name: str) -> PolicyDeclaration:
        try:
            return self._entries[name][1]
        except KeyError as exc:
            raise KeyError(f"unknown policy {name!r}") from exc

    def __iter__(self) -> Iterator[str]:
        return iter(self._entries)

