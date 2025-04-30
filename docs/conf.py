# Configuration file for the Sphinx documentation builder.

# -- Project information

project = 'MuJoCo Tools'
copyright = '2023-2025, Shanning Zhuang'
author = 'Shanning Zhuang'

release = '0.1'
version = '0.1.0'

# -- General configuration

extensions = [
    'sphinx.ext.duration',
    'sphinx.ext.doctest',
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.intersphinx',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
]

intersphinx_mapping = {
    'python': ('https://docs.python.org/3/', None),
    'sphinx': ('https://www.sphinx-doc.org/en/master/', None),
    'mujoco': ('https://mujoco.readthedocs.io/en/latest/', None),
}
intersphinx_disabled_domains = ['std']

templates_path = ['_templates']

# -- Options for HTML output

html_theme = "furo"
html_logo = "images/mujoco_tools_logo_jpg.jpg"
html_favicon = "images/mujoco_tools_logo_jpg.jpg"

# -- Options for autodoc
autodoc_member_order = 'bysource'
autodoc_typehints = 'description'
autoclass_content = 'both'

# -- Options for EPUB output
epub_show_urls = 'footnote'
