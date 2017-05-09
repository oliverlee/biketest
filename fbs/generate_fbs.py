#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from collections import OrderedDict
import os
import sys
import jinja2


def generate_source(filename, template_dict):
    template = template_setup(filename)
    filepath, in_ext = os.path.splitext(filename)
    assert in_ext == '.in', 'Invalid template file: {}'.format(filename)

    basename = os.path.basename(filepath)
    ext = os.path.splitext(basename)[1]
    source_dir = os.path.dirname(os.path.abspath(__file__))
    if ext == '.fbs':
        pass
    elif ext == '.py':
        source_dir = os.path.join(source_dir, os.pardir, 'python')
    elif ext == '.h':
        source_dir = os.path.join(source_dir, os.pardir, 'inc')
    else:
        raise NotImplementedError(
                'Source directory not defined for {} extensions.'.format(ext))
    out_filename = os.path.abspath(os.path.join(source_dir, basename))
    print("Generating file '{}'".format(out_filename))
    with open(out_filename, 'w') as f:
        f.write(template.render(template_dict))
        f.write('\n')


def template_setup(filename):
    template_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                'templates')
    template_loader = jinja2.FileSystemLoader(template_dir)
    template_env = jinja2.Environment(loader=template_loader)
    return template_env.get_template(filename);


if __name__ == "__main__":
    usage = 'generate flatbuffers schema files for given system size\n'
    usage += '{0} <n> <m> <l> <o> <p>'.format(__file__)
    usage += '\n    <n> - state size'
    usage += '\n    <m> - input size'
    usage += '\n    <l> - output size'
    usage += '\n    <o> - second order state size'
    usage += '\n    <p> - auxiliary state size'

    if len(sys.argv) < 5:
        print(usage)
        sys.exit(1)

    n = int(sys.argv[1])
    m = int(sys.argv[2])
    l = int(sys.argv[3])
    o = int(sys.argv[4])
    p = int(sys.argv[5])
    size = OrderedDict(zip(['n', 'm', 'l', 'o', 'p'], [n, m, l, o, p]))

    template_dict = {'size': size}
    generate_source('sample.fbs.in', template_dict)
    generate_source('sample_util.h.in', template_dict)
    generate_source('nptypes.py.in', template_dict)
