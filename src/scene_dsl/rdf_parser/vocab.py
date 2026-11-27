# SPDX-License-Identifier: MPL-2.0
from rdf_utils.namespace import URL_SECORO_MM
from rdflib import Namespace

NS_XML = Namespace("https://www.w3.org/TR/2006/REC-xml11-20060816#")
NS_URDF = Namespace("https://wiki.ros.org/urdf/XML/")
NS_MJCF = Namespace("https://mujoco.readthedocs.io/en/stable/XMLreference.html#")
NS_USD = Namespace("https://openusd.org/release/spec.html#")
NS_MM_ROS = Namespace("https://index.ros.org/p/")
NS_MM_BDD = Namespace(f"{URL_SECORO_MM}/acceptance-criteria/bdd#")
NS_MM_SENS = Namespace(f"{URL_SECORO_MM}/robot/sensors#")

URI_SENS_PRED_UPDATE_RATE = NS_MM_SENS["update-rate"]

# The friction of a collision shape's surface (newtonian-rigid-body-dynamics/contact.json).
NS_MM_CONTACT = Namespace(f"{URL_SECORO_MM}/newtonian-rigid-body-dynamics/contact#")
URI_CONTACT_TYPE_SURFACE_FRICTION = NS_MM_CONTACT["SurfaceFriction"]
URI_CONTACT_TYPE_COULOMB_FRICTION = NS_MM_CONTACT["CoulombFriction"]
URI_CONTACT_PRED_HAS_SURFACE_FRICTION = NS_MM_CONTACT["has-surface-friction"]
URI_CONTACT_PRED_SLIDING = NS_MM_CONTACT["sliding"]
URI_CONTACT_PRED_TORSIONAL = NS_MM_CONTACT["torsional"]
URI_CONTACT_PRED_ROLLING = NS_MM_CONTACT["rolling"]

URI_XML_DOCUMENT = NS_XML["document"]
URI_URDF_ROBOT = NS_URDF["robot"]
URI_MJCF_MUJOCO = NS_MJCF["mujoco"]
URI_USD_STAGE = NS_USD["stage"]

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
