tab = '  '


class XacroWriter():
    def __init__(self, file, robot_name):
        self.file = open(file, 'w+')
        self.initialize_file(robot_name)

    def write(self, string, indent_level=1):
        self.file.write('{0}{1}\n'.format(tab * indent_level, string))

    def write_include(self, package, file, path=None):
        self.write('<xacro:include filename="$(find {0})/{1}{2}.urdf.xacro"/>'.format(package, path, file))

    def write_extras(self, path):
        self.write_comment('Extras')
        self.write('<xacro:include filename="{0}" />'.format(path))

    def write_macro(self, macro, parameters=None, blocks=None):
        params = ''
        if parameters is not None:
            for p in parameters:
                params += ' {0}="{1}"'.format(p, parameters[p])

        if blocks is None:
            self.write('<xacro:{0}{1}/>\n'.format(macro, params))
        else:
            self.write('<xacro:{0}{1}>'.format(macro, params))
            self.write('{0}'.format(blocks), 2)
            self.write('</xacro:{0}>'.format(macro))

    def write_fixed_joint(self, name, parent, child, origin=None, indent_level=1):
        self.write('<joint name="{0}" type="fixed">'.format(name), indent_level)
        self.write('<parent link="{0}"/>'.format(parent), indent_level + 1)
        self.write('<child link="{0}"/>'.format(child), indent_level + 1)
        if origin is not None:
            self.write('{0}'.format(origin), indent_level + 1)
        self.write('</joint>', indent_level)

    def write_comment(self, comment, indent_level=1):
        self.write('<!-- {0} -->'.format(comment), indent_level)

    def write_newline(self):
        self.write('', 0)

    def add_origin(xyz, rpy):
        return '<origin xyz="{0}" rpy="{1}"/>'.format(str(xyz)[1:-1].replace(',', ''), str(rpy)[1:-1].replace(',', ''))

    def initialize_file(self, robot_name):
        self.write('<?xml version="1.0"?>', 0)
        self.write('<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="{0}">\n'.format(robot_name), 0)

    def close_file(self):
        self.write('</robot>', 0)
        self.file.close()