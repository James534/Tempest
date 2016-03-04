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
  OBJDIR     = obj/x64/Release/Test_Gwen_OpenGL
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/Test_Gwen_OpenGL_gmake_x64_release
  DEFINES   += -DGWEN_COMPILE_STATIC -D_HAS_EXCEPTIONS=0 -D_STATIC_CPPLIB -DDONT_USE_GLUT -DGLEW_STATIC -DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1
  INCLUDES  += -I../../examples/ThirdPartyLibs -I../../examples -I../../test/GwenOpenGLTest -I../../examples/ThirdPartyLibs/Glew
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -O2 -msse2 -ffast-math -m64
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -s -m64 -L/usr/lib64
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += ../../bin/libgwen_gmake_x64_release.a -lGL -ldl -lX11 -lpthread
  LDDEPS    += ../../bin/libgwen_gmake_x64_release.a
  LINKCMD    = $(CXX) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),debug64)
  OBJDIR     = obj/x64/Debug/Test_Gwen_OpenGL
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/Test_Gwen_OpenGL_gmake_x64_debug
  DEFINES   += -D_DEBUG=1 -DGWEN_COMPILE_STATIC -D_HAS_EXCEPTIONS=0 -D_STATIC_CPPLIB -DDONT_USE_GLUT -DGLEW_STATIC -DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1
  INCLUDES  += -I../../examples/ThirdPartyLibs -I../../examples -I../../test/GwenOpenGLTest -I../../examples/ThirdPartyLibs/Glew
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -g -ffast-math -m64
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -m64 -L/usr/lib64
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += ../../bin/libgwen_gmake_x64_debug.a -lGL -ldl -lX11 -lpthread
  LDDEPS    += ../../bin/libgwen_gmake_x64_debug.a
  LINKCMD    = $(CXX) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

OBJECTS := \
	$(OBJDIR)/glew.o \
	$(OBJDIR)/OpenSans.o \
	$(OBJDIR)/TwFonts.o \
	$(OBJDIR)/LoadShader.o \
	$(OBJDIR)/GLPrimitiveRenderer.o \
	$(OBJDIR)/fontstash.o \
	$(OBJDIR)/opengl_fontstashcallbacks.o \
	$(OBJDIR)/b3Clock.o \
	$(OBJDIR)/OpenGLSample.o \
	$(OBJDIR)/CrossSplitter.o \
	$(OBJDIR)/GroupBox.o \
	$(OBJDIR)/Checkbox.o \
	$(OBJDIR)/Numeric.o \
	$(OBJDIR)/MenuStrip.o \
	$(OBJDIR)/ImagePanel.o \
	$(OBJDIR)/Slider.o \
	$(OBJDIR)/Label.o \
	$(OBJDIR)/TextBox.o \
	$(OBJDIR)/Button.o \
	$(OBJDIR)/RadioButton.o \
	$(OBJDIR)/Properties.o \
	$(OBJDIR)/PanelListPanel.o \
	$(OBJDIR)/UnitTest.o \
	$(OBJDIR)/ListBox.o \
	$(OBJDIR)/TabControl.o \
	$(OBJDIR)/TreeControl.o \
	$(OBJDIR)/ComboBox.o \
	$(OBJDIR)/ScrollControl.o \
	$(OBJDIR)/ProgressBar.o \
	$(OBJDIR)/StatusBar.o \
	$(OBJDIR)/X11OpenGLWindow.o \

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
	@echo Linking Test_Gwen_OpenGL
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
	@echo Cleaning Test_Gwen_OpenGL
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

$(OBJDIR)/glew.o: ../../examples/ThirdPartyLibs/Glew/glew.c
	@echo $(notdir $<)
	$(SILENT) $(CC) $(CFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/OpenSans.o: ../../examples/OpenGLWindow/OpenSans.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/TwFonts.o: ../../examples/OpenGLWindow/TwFonts.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/LoadShader.o: ../../examples/OpenGLWindow/LoadShader.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/GLPrimitiveRenderer.o: ../../examples/OpenGLWindow/GLPrimitiveRenderer.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/fontstash.o: ../../examples/OpenGLWindow/fontstash.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/opengl_fontstashcallbacks.o: ../../examples/OpenGLWindow/opengl_fontstashcallbacks.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/b3Clock.o: ../../examples/Utils/b3Clock.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/OpenGLSample.o: ../../test/GwenOpenGLTest/OpenGLSample.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/CrossSplitter.o: ../../test/GwenOpenGLTest/CrossSplitter.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/GroupBox.o: ../../test/GwenOpenGLTest/GroupBox.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/Checkbox.o: ../../test/GwenOpenGLTest/Checkbox.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/Numeric.o: ../../test/GwenOpenGLTest/Numeric.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/MenuStrip.o: ../../test/GwenOpenGLTest/MenuStrip.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/ImagePanel.o: ../../test/GwenOpenGLTest/ImagePanel.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/Slider.o: ../../test/GwenOpenGLTest/Slider.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/Label.o: ../../test/GwenOpenGLTest/Label.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/TextBox.o: ../../test/GwenOpenGLTest/TextBox.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/Button.o: ../../test/GwenOpenGLTest/Button.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/RadioButton.o: ../../test/GwenOpenGLTest/RadioButton.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/Properties.o: ../../test/GwenOpenGLTest/Properties.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/PanelListPanel.o: ../../test/GwenOpenGLTest/PanelListPanel.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/UnitTest.o: ../../test/GwenOpenGLTest/UnitTest.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/ListBox.o: ../../test/GwenOpenGLTest/ListBox.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/TabControl.o: ../../test/GwenOpenGLTest/TabControl.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/TreeControl.o: ../../test/GwenOpenGLTest/TreeControl.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/ComboBox.o: ../../test/GwenOpenGLTest/ComboBox.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/ScrollControl.o: ../../test/GwenOpenGLTest/ScrollControl.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/ProgressBar.o: ../../test/GwenOpenGLTest/ProgressBar.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/StatusBar.o: ../../test/GwenOpenGLTest/StatusBar.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/X11OpenGLWindow.o: ../../examples/OpenGLWindow/X11OpenGLWindow.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"

-include $(OBJECTS:%.o=%.d)
