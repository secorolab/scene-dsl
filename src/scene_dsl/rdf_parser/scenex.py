# SPDX-License-Identifier: MPL-2.0
from bdd_dsl.execution.common import load_attr_path
from rdflib import Graph, URIRef
from rdf_utils.models.common import ModelBase

from scene_dsl.rdf.scenex import URI_EXEC_PRED_MODEL, URI_EXEC_TYPE_SCENE_INST


class SceneInstanceModel(ModelBase):
    models: dict[URIRef, ModelBase]

    def __init__(self, scn_inst_id: URIRef, graph: Graph) -> None:
        super().__init__(node_id=scn_inst_id, graph=graph)
        assert URI_EXEC_TYPE_SCENE_INST in self.types, (
            f"node '{scn_inst_id}' is not a scene instance"
        )

        self.models = {}
        for model_id in graph.objects(subject=scn_inst_id, predicate=URI_EXEC_PRED_MODEL):
            assert isinstance(model_id, URIRef), (
                f"unexpected scene model ID type: {type(model_id)}"
            )
            model = ModelBase(node_id=model_id, graph=graph)
            assert model_id not in self.models, (
                f"scene instance '{self.id}' has duplicate model '{model_id}'"
            )
            load_attr_path(graph=graph, model=model)
            self.models[model_id] = model

        assert self.models, f"scene instance '{self.id}' has no model"
