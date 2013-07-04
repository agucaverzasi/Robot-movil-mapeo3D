FILE(REMOVE_RECURSE
  "../bin/planner.pdb"
  "../bin/planner"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/planner.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
