import os

def create_component(category, component_name, project_root="."):
    
    existing_categories = ["brain", "drivers", "periodics", "utils"]

    path_EmbeddedPlatform = str(os.getcwd())
    last_backslash = str(os.getcwd())[::-1].index('\\')
    filename_Emb = path_EmbeddedPlatform[:len(path_EmbeddedPlatform)-last_backslash]
    filename_CMake = path_EmbeddedPlatform[:len(path_EmbeddedPlatform)-last_backslash] + "CMakeLists.txt"
    
    if category not in existing_categories:
        last_include = -1
        last_source = -1

        with open(filename_CMake, 'r') as file:
            lines = file.readlines()

        for x in lines:
            if '    ${CMAKE_CURRENT_SOURCE_DIR}/include/' in x:
                last_include = int(lines.index(x))
            if '    "${CMAKE_CURRENT_SOURCE_DIR}/source/' in x:
                last_source = int(lines.index(x))

        lines.insert(last_source + 1, '    "${CMAKE_CURRENT_SOURCE_DIR}/source/' + category + '/*.cpp"' + '\n')
        lines.insert(last_include + 2, '    ${CMAKE_CURRENT_SOURCE_DIR}/include/' + category + '\n')

        with open(filename_CMake, 'w') as file:
            file.writelines(lines)

    include_path = os.path.join(filename_Emb, "include", category)
    source_path = os.path.join(filename_Emb, "source", category)

    os.makedirs(include_path, exist_ok=True)
    os.makedirs(source_path, exist_ok=True)

    #Create header file
    header_file_path = os.path.join(include_path, f"{component_name}.hpp")
    if not os.path.exists(header_file_path):
        with open(header_file_path, "w") as f:
            f.write(f"#ifndef {component_name.upper()}_HPP\n")
            f.write(f"#define {component_name.upper()}_HPP\n\n")
            f.write(f"// TODO: Add your code here\n\n")
            f.write(f"namespace {category}\n")
            f.write('{\n' + '   /**\n')
            f.write('    * @brief Class ' + component_name + '\n' + '    *\n' + '    */\n')
            f.write(f'    class C{component_name.capitalize()}\n' + '    {\n' + '        public:\n' + '            /* Construnctor */\n' + f'            C{component_name.capitalize()}(\n')
            f.write('                \n' + '            );\n' + '            /* Destructor */\n' + f'            ~C{component_name.capitalize()}();\n')
            f.write('        private:\n' + '            /* private variables & method member */\n')
            f.write('    }; // class ' + f'C{component_name.capitalize()}\n')
            f.write('}; // namespace '+ f'{category}\n\n')
            f.write(f"#endif // {component_name.upper()}_HPP\n")
        print(f"Created: {header_file_path}")
    else:
        print(f"File already exists: {header_file_path}")

    #Create source file
    source_file_path = os.path.join(source_path, f"{component_name}.cpp")
    if not os.path.exists(source_file_path):
        with open(source_file_path, "w") as f:
            f.write(f'#include "{category}/{component_name}.hpp"\n\n')
            f.write(f"// TODO: Add your code here\n")
            f.write(f"namespace {category}\n")
            f.write('{\n' + '   /**\n')
            f.write('    * @brief Class constructor' + component_name + '\n' + '    *\n' + '    */\n')
            f.write(f'    C{component_name.capitalize()}::C{component_name.capitalize()}()\n' + '    {\n')
            f.write('        /* constructor behaviour */\n' + '    }\n' + '\n')
            f.write(f'    /** @brief  C{component_name.capitalize()} class destructor\n' + '     */\n')
            f.write(f'    C{component_name.capitalize()}::~C{component_name.capitalize()}()\n' + '    {\n' + '    };\n' + '\n' + '}; // namespace periodics')
        print(f"Created: {source_file_path}")
    else:
        print(f"File already exists: {source_file_path}")

category = input("Type the category of the new component (brain, drivers, periodics, utils or the name for the new one): ").strip().lower()
component_name = input("Type the name of the new component: ").strip().lower()

if component_name[0].isalpha():
    create_component(category, component_name)
else:
    print("Invalid component name")