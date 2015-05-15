# add option --gen-suppressions=yes to get data to add to suppression file
valgrind --leak-check=full --suppressions=suppression.valgrind --track-origins=yes $*
