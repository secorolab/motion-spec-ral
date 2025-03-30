# SPDX-License-Identifier: MPL-2.0
import sys
import rdflib
import pyshacl
import resolver
from functools import reduce
import operator
from namespace import APP


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(0)

    app_model = sys.argv[1]

    # Load top-level, application model
    g = rdflib.ConjunctiveGraph()
    g.parse(app_model, format="json-ld")

    # Load IRI map
    url_map = {}
    for key in g.objects(predicate=APP["iri-map"]):
        value = g.value(key, APP["path"]).value
        url_map[str(key)] = value

    resolver.install(resolver.IriToFileResolver(url_map))

    # Load/import the referenced models
    models = list(g.objects(predicate=APP["import"]))
    for o in models:
        g.parse(location=o, format="json-ld")

    g_sh = rdflib.ConjunctiveGraph()
    metamodels = list(g.objects(predicate=APP["constraints"]))
    for o in metamodels:
        path = resolver.map(url_map, o) or o
        g_sh.parse(location=path, format="turtle")

    # Flatten the conjunctive graph into a graph.
    # This is required because in rdflib/PySHACL links can only be traversed
    # when both, the subject and the object are part of the same graph. In other
    # words, such constraints won't validate in datasets.
    # cf. <https://github.com/RDFLib/pySHACL/issues/152>
    g_flat = reduce(operator.iadd, g.contexts(), rdflib.Graph())

    conforms, v_graph, v_text = pyshacl.validate(
            data_graph=g_flat,
            shacl_graph=g_sh,
            inference="none",
            meta_shacl=True)

    print(v_text)
