# SPDX-License-Identifier: MPL-2.0
from rdflib import Namespace

NS_XML = Namespace("https://www.w3.org/TR/2006/REC-xml11-20060816#")
NS_URDF = Namespace("https://wiki.ros.org/urdf/XML/")
NS_MJCF = Namespace("https://mujoco.readthedocs.io/en/stable/XMLreference.html#")
NS_USD = Namespace("https://openusd.org/release/spec.html#")
NS_MM_ROS = Namespace("https://index.ros.org/p/")

URI_XML_DOCUMENT = NS_XML["document"]
URI_URDF_ROBOT = NS_URDF["robot"]
URI_MJCF_MUJOCO = NS_MJCF["mujoco"]
URI_USD_STAGE = NS_USD["stage"]

URI_ROS_TYPE_PACKAGE = NS_MM_ROS["Package"]
URI_ROS_PRED_PACKAGE_NAME = NS_MM_ROS["package-name"]
