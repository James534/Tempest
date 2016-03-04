# GNU Make project makefile autogenerated by Premake
ifndef config
  config=release64
endif

ifndef verbose
  SILENT = @
endif

ifndef CC
  CC = gcc
endif

ifndef CXX
  CXX = g++
endif

ifndef AR
  AR = ar
endif

ifndef RESCOMP
  ifdef WINDRES
    RESCOMP = $(WINDRES)
  else
    RESCOMP = windres
  endif
endif

ifeq ($(config),release64)
  OBJDIR     = obj/x64/Release/Test_PhysicsServerLoopBack
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/Test_PhysicsServerLoopBack_gmake_x64_release
  DEFINES   += -DPHYSICS_LOOP_BACK
  INCLUDES  += -I../../src -I../../examples -I../../examples/ThirdPartyLibs
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -O2 -msse2 -ffast-math -m64
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -s -m64 -L/usr/lib64
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += ../../bin/libBulletFileLoader_gmake_x64_release.a ../../bin/libBulletWorldImporter_gmake_x64_release.a ../../bin/libBullet3Common_gmake_x64_release.a ../../bin/libBulletDynamics_gmake_x64_release.a ../../bin/libBulletCollision_gmake_x64_release.a ../../bin/libLinearMath_gmake_x64_release.a
  LDDEPS    += ../../bin/libBulletFileLoader_gmake_x64_release.a ../../bin/libBulletWorldImporter_gmake_x64_release.a ../../bin/libBullet3Common_gmake_x64_release.a ../../bin/libBulletDynamics_gmake_x64_release.a ../../bin/libBulletCollision_gmake_x64_release.a ../../bin/libLinearMath_gmake_x64_release.a
  LINKCMD    = $(CXX) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),debug64)
  OBJDIR     = obj/x64/Debug/Test_PhysicsServerLoopBack
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/Test_PhysicsServerLoopBack_gmake_x64_debug
  DEFINES   += -D_DEBUG=1 -DPHYSICS_LOOP_BACK
  INCLUDES  += -I../../src -I../../examples -I../../examples/ThirdPartyLibs
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -g -ffast-math -m64
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -m64 -L/usr/lib64
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += ../../bin/libBulletFileLoader_gmake_x64_debug.a ../../bin/libBulletWorldImporter_gmake_x64_debug.a ../../bin/libBullet3Common_gmake_x64_debug.a ../../bin/libBulletDynamics_gmake_x64_debug.a ../../bin/libBulletCollision_gmake_x64_debug.a ../../bin/libLinearMath_gmake_x64_debug.a
  LDDEPS    += ../../bin/libBulletFileLoader_gmake_x64_debug.a ../../bin/libBulletWorldImporter_gmake_x64_debug.a ../../bin/libBullet3Common_gmake_x64_debug.a ../../bin/libBulletDynamics_gmake_x64_debug.a ../../bin/libBulletCollision_gmake_x64_debug.a ../../bin/libLinearMath_gmake_x64_debug.a
  LINKCMD    = $(CXX) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

OBJECTS := \
	$(OBJDIR)/test.o \
	$(OBJDIR)/PhysicsClient.o \
	$(OBJDIR)/PhysicsServer.o \
	$(OBJDIR)/PhysicsServerSharedMemory.o \
	$(OBJDIR)/PhysicsServerCommandProcessor.o \
	$(OBJDIR)/PhysicsLoopBack.o \
	$(OBJDIR)/PhysicsLoopBackC_API.o \
	$(OBJDIR)/PhysicsClientSharedMemory.o \
	$(OBJDIR)/PhysicsClientC_API.o \
	$(OBJDIR)/Win32SharedMemory.o \
	$(OBJDIR)/PosixSharedMemory.o \
	$(OBJDIR)/b3ResourcePath.o \
	$(OBJDIR)/tinystr.o \
	$(OBJDIR)/tinyxml.o \
	$(OBJDIR)/tinyxmlerror.o \
	$(OBJDIR)/tinyxmlparser.o \
	$(OBJDIR)/tiny_obj_loader.o \
	$(OBJDIR)/LoadMeshFromCollada.o \
	$(OBJDIR)/LoadMeshFromObj.o \
	$(OBJDIR)/Wavefront2GLInstanceGraphicsShape.o \
	$(OBJDIR)/BulletUrdfImporter.o \
	$(OBJDIR)/MyMultiBodyCreator.o \
	$(OBJDIR)/URDF2Bullet.o \
	$(OBJDIR)/UrdfParser.o \
	$(OBJDIR)/urdfStringSplit.o \

RESOURCES := \

SHELLTYPE := msdos
ifeq (,$(ComSpec)$(COMSPEC))
  SHELLTYPE := posix
endif
ifeq (/bin,$(findstring /bin,$(SHELL)))
  SHELLTYPE := posix
endif

.PHONY: clean prebuild prelink

all: $(TARGETDIR) $(OBJDIR) prebuild prelink $(TARGET)
	@:

$(TARGET): $(GCH) $(OBJECTS) $(LDDEPS) $(RESOURCES)
	@echo Linking Test_PhysicsServerLoopBack
	$(SILENT) $(LINKCMD)
	$(POSTBUILDCMDS)

$(TARGETDIR):
	@echo Creating $(TARGETDIR)
ifeq (posix,$(SHELLTYPE))
	$(SILENT) mkdir -p $(TARGETDIR)
else
	$(SILENT) mkdir $(subst /,\\,$(TARGETDIR))
endif

$(OBJDIR):
	@echo Creating $(OBJDIR)
ifeq (posix,$(SHELLTYPE))
	$(SILENT) mkdir -p $(OBJDIR)
else
	$(SILENT) mkdir $(subst /,\\,$(OBJDIR))
endif

clean:
	@echo Cleaning Test_PhysicsServerLoopBack
ifeq (posix,$(SHELLTYPE))
	$(SILENT) rm -f  $(TARGET)
	$(SILENT) rm -rf $(OBJDIR)
else
	$(SILENT) if exist $(subst /,\\,$(TARGET)) del $(subst /,\\,$(TARGET))
	$(SILENT) if exist $(subst /,\\,$(OBJDIR)) rmdir /s /q $(subst /,\\,$(OBJDIR))
endif

prebuild:
	$(PREBUILDCMDS)

prelink:
	$(PRELINKCMDS)

ifneq (,$(PCH))
$(GCH): $(PCH)
	@echo $(notdir $<)
ifeq (posix,$(SHELLTYPE))
	-$(SILENT) cp $< $(OBJDIR)
else
	$(SILENT) xcopy /D /Y /Q "$(subst /,\,$<)" "$(subst /,\,$(OBJDIR))" 1>nul
endif
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
endif

$(OBJDIR)/test.o: ../../test/SharedMemory/test.c
	@echo $(notdir $<)
	$(SILENT) $(CC) $(CFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/PhysicsClient.o: ../../examples/SharedMemory/PhysicsClient.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/PhysicsServer.o: ../../examples/SharedMemory/PhysicsServer.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/PhysicsServerSharedMemory.o: ../../examples/SharedMemory/PhysicsServerSharedMemory.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/PhysicsServerCommandProcessor.o: ../../examples/SharedMemory/PhysicsServerCommandProcessor.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/PhysicsLoopBack.o: ../../examples/SharedMemory/PhysicsLoopBack.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/PhysicsLoopBackC_API.o: ../../examples/SharedMemory/PhysicsLoopBackC_API.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/PhysicsClientSharedMemory.o: ../../examples/SharedMemory/PhysicsClientSharedMemory.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/PhysicsClientC_API.o: ../../examples/SharedMemory/PhysicsClientC_API.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/Win32SharedMemory.o: ../../examples/SharedMemory/Win32SharedMemory.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/PosixSharedMemory.o: ../../examples/SharedMemory/PosixSharedMemory.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/b3ResourcePath.o: ../../examples/Utils/b3ResourcePath.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/tinystr.o: ../../examples/ThirdPartyLibs/tinyxml/tinystr.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/tinyxml.o: ../../examples/ThirdPartyLibs/tinyxml/tinyxml.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/tinyxmlerror.o: ../../examples/ThirdPartyLibs/tinyxml/tinyxmlerror.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/tinyxmlparser.o: ../../examples/ThirdPartyLibs/tinyxml/tinyxmlparser.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/tiny_obj_loader.o: ../../examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/LoadMeshFromCollada.o: ../../examples/Importers/ImportColladaDemo/LoadMeshFromCollada.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/LoadMeshFromObj.o: ../../examples/Importers/ImportObjDemo/LoadMeshFromObj.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/Wavefront2GLInstanceGraphicsShape.o: ../../examples/Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/BulletUrdfImporter.o: ../../examples/Importers/ImportURDFDemo/BulletUrdfImporter.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/MyMultiBodyCreator.o: ../../examples/Importers/ImportURDFDemo/MyMultiBodyCreator.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/URDF2Bullet.o: ../../examples/Importers/ImportURDFDemo/URDF2Bullet.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/UrdfParser.o: ../../examples/Importers/ImportURDFDemo/UrdfParser.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/urdfStringSplit.o: ../../examples/Importers/ImportURDFDemo/urdfStringSplit.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"

-include $(OBJECTS:%.o=%.d)
