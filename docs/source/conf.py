from pathlib import Path
import sys

project = 'Forklift Gazebo'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration


extensions = [
    'myst_parser',
    'sphinx.ext.viewcode',
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.napoleon',
    'sphinx_rtd_theme',
    'sphinxcontrib.mermaid',
]
myst_heading_anchors = 4

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']


# use concise bullet point class variable list
napoleon_use_ivar = True

# Support use of sections called "Attrs" as a "Attributes" shorthand
napoleon_custom_sections = [
    ("Attrs", "Parameters"),
    ("Kwargs", "Keyword Arguments"),
    ("Encoding", "params_style")
]

autodoc_typehints = "signature"

napoleon_include_init_with_doc = True

# display the class signature as a method rather than in the name
autodoc_class_signature = "separated"

# doc method by source order
autodoc_member_order = "bysource"
