# SPDX-License-Identifier: MPL-2.0
from rdf_utils.namespace import NS_MM_KC_EXT, URL_SECORO_MM
from rdflib import Namespace

NS_XML = Namespace("https://www.w3.org/TR/2006/REC-xml11-20060816#")
NS_URDF = Namespace("https://wiki.ros.org/urdf/XML/")
NS_MJCF = Namespace("https://mujoco.readthedocs.io/en/stable/XMLreference.html#")
NS_USD = Namespace("https://openusd.org/release/spec.html#")
NS_MM_ROS = Namespace("https://index.ros.org/p/")
NS_MM_BDD = Namespace(f"{URL_SECORO_MM}/acceptance-criteria/bdd#")

# What a tree or a graph is built from: the trees below it, and the bodies it hangs
# itself. The DSL's `tree <x>` and a graph's own bodies would otherwise leave no trace.
URI_KC_EXT_PRED_COMPOSES = NS_MM_KC_EXT["composes"]

URI_XML_DOCUMENT = NS_XML["document"]
URI_URDF_ROBOT = NS_URDF["robot"]
URI_MJCF_MUJOCO = NS_MJCF["mujoco"]
URI_USD_STAGE = NS_USD["stage"]

URI_ROS_TYPE_PACKAGE = NS_MM_ROS["Package"]
URI_ROS_PRED_PACKAGE_NAME = NS_MM_ROS["package-name"]

# Scene DSL historically uses these stable BDD vocabulary IRIs. They live here so
# scene-dsl owns its RDF representation without depending on bdd-dsl.
URI_BDD_TYPE_SCENE = NS_MM_BDD["Scene"]
URI_BDD_TYPE_SCENE_OBJ = NS_MM_BDD["SceneHasObjects"]
URI_BDD_TYPE_SCENE_WS = NS_MM_BDD["SceneHasWorkspaces"]
URI_BDD_TYPE_SCENE_AGN = NS_MM_BDD["SceneHasAgents"]
URI_BDD_TYPE_SET = NS_MM_BDD["Set"]
URI_BDD_TYPE_CONST_SET = NS_MM_BDD["ConstantSet"]
URI_BDD_PRED_ELEMS = NS_MM_BDD["elements"]
URI_BDD_PRED_OF_SCENE = NS_MM_BDD["of-scene"]
