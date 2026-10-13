CXX = g++
#CXXFLAGS = -std=c++20 -Wall -Wextra -I/usr/include/eigen3 
EIGEN_DIR ?= C:\Users\z004ryjh\Desktop\GIT\FEA\FEMSolver\third_party\eigen-5.0.0

# Preprocessor options
EIGEN_CPPFLAGS += -I$(EIGEN_DIR)
# Compiler options
CXXFLAGS = -std=c++20 -Wall -Wextra $(EIGEN_CPPFLAGS)

BUILD_DIR = build
TARGET := $(BUILD_DIR)/solver.exe

SOURCES = src/main.cpp $(wildcard src/fem/*.cpp)
OBJECTS = $(patsubst src/%.cpp,$(BUILD_DIR)/%.o, $(SOURCES))
DEPS := $(OBJECTS:.o=.d)

#'all' & 'clean' are made prerequisites of the special target phony. Otherwise, a file named 
#'clean' might hinder the execution of the clean recipe, which has not prerequisites.
# Without prerequisites, a recipe is not executed. Recipes are only ever executed 
# if their prerequisites have been changed.
# A phony target is a target that is always out-of-date. It will run no matter the state of the
# system.
.PHONY: all install clean distclean TAGS info check
.DEFAULT_GOAL := all

#defines the default goal: as we run make without specifying any target
all: $(TARGET)

# TARGET: the file Make should produce.
# OBJECTS: Normal Prerequisite 
#		 - object files that must be compiled before the executable can be linked.
# BUILD: Order-only Prerequisite
#		 - the build/ directory must exist before the linking command runs
#        - changes to the directory's timestamp alone should not trigger relinking.
#          -> creating a random file in this directory should not trigger relinking.
$(TARGET): $(OBJECTS) | $(BUILD_DIR)
	$(CXX) $(OBJECTS) -o $@

# Compile sources and generate header dependencies.
# dir extracts the directory portion of a path.
# -MD generates a dependency file listing the user headers included by the successive 
#     source files.
# -MP adds dummy targets for the headers previously generated in order to remove 
#     errors when headers are removed or renamed.
#     The dummy targets inform the compiler that the headers have rules, even if the 
#     corresponding files have been removed.
#     In this way, the compilation does not fail right away because of a reference to 
#     a deleted header file in a .d file.
#     .d files get re-generated every time the corresponding .cpp file is recompiled.
#     Therefore, this flag allows you to regenerate the .d files with mentions to deleted 
#     headers. Otherwise, make would fail before reaching that point, unless you manually 
#     delete the .d files, or to delete them all in order to re-generate them.
# -MF specififes the dependency files path
# $*  automatic variable that represents the portion matched by %
$(BUILD_DIR)/%.o: src/%.cpp
	mkdir -p $(dir $@)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -MMD -MP \
		-MF $(BUILD_DIR)/$*.d \
		-c $< -o $@

# Create the main build directory.
$(BUILD_DIR):
	mkdir -p $@


# Include generated dependency files, if present.
-include $(DEPS)

# Remove all generated build artifacts.
clean:
	rm -f $(OBJECTS) $(TARGET) *.exe
	rm -rf $(BUILD_DIR)
