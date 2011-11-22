get_directory_property(inc_dirs INCLUDE_DIRECTORIES)

set(filename includepaths.txt)
file(WRITE  ${filename} "copy this to .cproject\n\n")

foreach(includeLine ${inc_dirs})
  file(APPEND ${filename} "<listOptionValue builtIn=\"false\" value=\"${includeLine}\"/>\n")
endforeach(includeLine ${inc_dirs})