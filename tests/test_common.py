from pathlib import Path


MODELS_DIR = Path(__file__).parents[1] / "examples" / "models"


def write_example_scene(tmp_path: Path) -> None:
    (tmp_path / "example.scene").write_text(
        """ns n = "https://example.test/"
obj set (ns=n) objs { object cup, object bowl }
ws set (ns=n) wss { workspace table }
agn set (ns=n) agns { agent robot }
scene (ns=n) s {
    obj set <objs>
    ws set <wss>
    agn set <agns>
}
"""
    )
