#! /usr/bin/env python3


def shave(xl, xu, yl, yu, node_in, edge_in, node_out, edge_out):
    node_file = open(node_in)
    graph_name = node_file.readline().strip()
    node_file.readline()  # dump number of vertices

    new_graph_vertices = ""
    new_nodes = set()
    nodes = list()

    # read vertices
    for line in node_file.readlines():
        id_, x, y = line.split()
        if xl < float(x) < xu and yl < float(y) < yu:
            new_nodes.add(id_)
            nodes.append([id_,x,y])
            new_graph_vertices += (id_ + " " + x + " " + y + "\n")

    node_file.close()

    edge_file = open(edge_in)
    edge_file.readline()  # dump number of edges

    new_graph_edges = ""
    for line in edge_file.readlines():
        id_, u, v, length = line.split()
        if new_nodes.__contains__(u) and new_nodes.__contains__(v):
            new_graph_edges += (id_ + " " + u + " " + v + " " + length + "\n")

    edge_file.close()

    # put nodes into new file
    new_node_file = open(node_out, "w")
    new_node_file.write(graph_name + "\n")
    new_node_file.write(str(len(nodes)) + "\n")
    new_node_file.write(new_graph_vertices)
    new_node_file.close()

    # put edges into new file
    new_edge_file = open(edge_out, "w")
    new_edge_file.write(str(len(new_graph_edges)) + "\n")
    new_edge_file.write(new_graph_edges)
    new_edge_file.close()


def get_nodes_in(xl, xu, yl, yu, node_file):
    f = open(node_file)
    f.readline()  # dump graph name
    f.readline()  # dump nr vertices

    for line in f.readlines():
        id_, x, y = line.split()
        if xl < float(x) < xu and yl < float(y) < yu:
            print("node ", id_, " on pos ", x, ":", y)


def to_edge_based(node_file, edge_file, output_file):
    f = open(node_file)
    g_name = f.readline().strip()
    f.readline()

    n_ids = {}
    n_id_edges = {}
    n_id_gps = {}

    i = 0
    for line in f.readlines():
        id_, x, y = line.split()
        n_ids[id_] = i
        n_id_edges[n_ids[id_]] = []
        n_id_gps[n_ids[id_]] = (x,y)
        i += 1

    f.close()

    f = open(edge_file)
    f.readline()  # nr of edges

    e_ids = {}
    e_id_len = {}
    e_neighbs = {}
    e_id_gps = {}

    i = 0
    for line in f.readlines():
        ide, fr, t, weight = line.split()
        e_ids[ide] = i
        e_id_len[e_ids[ide]] = weight
        e_neighbs[e_ids[ide]] = []
        e_id_gps[e_ids[ide]] = []
        # each node has id of edge coming in/out
        n_id_edges[n_ids[fr]].append(e_ids[ide])
        n_id_edges[n_ids[t]].append(e_ids[ide])
        i += 1

    f.close()

    for n in n_id_edges:
        for e1 in n_id_edges[n]:
            for e2 in n_id_edges[n]:
                if e1 != e2:
                    e_neighbs[e1].append(e2)
            e_id_gps[e1].append(n_id_gps[n])

    f2 = open(output_file, "w")
    for e in e_neighbs:
        f2.write(str(e) + " " + str(e_id_len[e]) + " ")
        # write gpses of edges end points
        for gps in e_id_gps[e]:
            f2.write(str(gps[0]) + " " + str(gps[1]) + " ")
        # write all neighbours
        for ed in e_neighbs[e]:
            f2.write(str(ed) + " ")
        f2.write("\n")

    f2.close()
    print("Done.")


# shave(0,4000,0,4000,
#       "/home/adam/school/bakalarka/prakticka/data/san_fran_nodes.txt",
#       "/home/adam/school/bakalarka/prakticka/data/san_fran_edges.txt",
#       "/home/adam/school/bakalarka/prakticka/data/shaved_sf_nodes.txt",
#       "/home/adam/school/bakalarka/prakticka/data/shaved_sf_edges.txt")


# get_nodes_in(0, 4000, 0, 4000, "/home/adam/school/bakalarka/prakticka/data/shaved_sf_nodes.txt")

# to_edge_based(
#     "/home/adam/school/bakalarka/prakticka/data/shaved_sf_nodes.txt",
#     "/home/adam/school/bakalarka/prakticka/data/shaved_sf_edges.txt",
#     # "/home/adam/school/bakalarka/prakticka/data/.txt"
#     "/home/adam/Desktop/shaved_sf_edges_only.txt"
# )

# to_edge_based(
#     "/home/adam/school/bakalarka/prakticka/data/san_fran_nodes.txt",
#     "/home/adam/school/bakalarka/prakticka/data/san_fran_edges.txt",
#     "/home/adam/Desktop/sf_edges_only.txt"
# )

shave(6715,7060,4330,4700,
      "/home/adam/school/bakalarka/prakticka/data/san_fran_nodes.txt",
      "/home/adam/school/bakalarka/prakticka/data/san_fran_edges.txt",
      "/home/adam/Desktop/very_shaved_sf_nodes.txt",
      "/home/adam/Desktop/very_shaved_sf_edges.txt")

to_edge_based(
    "/home/adam/Desktop/very_shaved_sf_nodes.txt",
    "/home/adam/Desktop/very_shaved_sf_edges.txt",
    "/home/adam/Desktop/sf1k.txt"
)


