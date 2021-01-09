ifeq ($(LLVM_SUPPORT),1)
LLVM_FLAGS := -target arm-linux-androideabi
endif

$(BUILDDIR)/%.o: %.c $(SRCDEPS)
	@$(MKDIR)
	@echo compiling $<
	$(NOECHO)$(CC) $(LLVM_FLAGS) $(CFLAGS) $(THUMBCFLAGS) --std=c99 $(INCLUDES) -c $< -MD -MT $@ -MF $(@:%o=%d) -o $@

$(BUILDDIR)/%.o: %.cpp $(SRCDEPS)
	@$(MKDIR)
	@echo compiling $<
	$(NOECHO)$(CC) $(LLVM_FLAGS) $(CFLAGS) $(CPPFLAGS) $(THUMBCFLAGS) $(INCLUDES) -c $< -MD -MT $@ -MF $(@:%o=%d) -o $@

# to override thumb setting, mark the .o file as .Ao
$(BUILDDIR)/%.Ao: %.c $(SRCDEPS)
	@$(MKDIR)
	@echo compiling $<
	$(NOECHO)$(CC) $(LLVM_FLAGS) $(CFLAGS) --std=c99 $(INCLUDES) -c $< -MD -MT $@ -MF $(@:%o=%d) -o $@

$(BUILDDIR)/%.Ao: %.cpp $(SRCDEPS)
	@$(MKDIR)
	@echo compiling $<
	$(NOECHO)$(CC) $(LLVM_FLAGS) $(CFLAGS) $(CPPFLAGS) $(INCLUDES) -c $< -MD -MT $@ -MF $(@:%o=%d) -o $@

# assembly is always compiled in ARM mode at the moment
$(BUILDDIR)/%.Ao: %.S $(SRCDEPS)
	@$(MKDIR)
	@echo compiling $<
	$(NOECHO)$(CC) $(LLVM_FLAGS) $(CFLAGS) $(ASMFLAGS) $(INCLUDES) -c $< -MD -MT $@ -MF $(@:%o=%d) -o $@

ifeq ($(ENABLE_TRUSTZONE), 1)
$(BUILDDIR)/%.o: %.S $(SRCDEPS)
	@$(MKDIR)
	@echo compiling $<
	$(NOECHO)$(CC) $(LLVM_FLAGS) -DENABLE_TRUSTZONE $(CFLAGS) $(ASMFLAGS) $(INCLUDES) -c $< -MD -MT $@ -MF $(@:%o=%d) -o $@
else
$(BUILDDIR)/%.o: %.S $(SRCDEPS)
	@$(MKDIR)
	@echo compiling $<
	$(NOECHO)$(CC) $(LLVM_FLAGS) $(CFLAGS) $(ASMFLAGS) $(INCLUDES) -c $< -MD -MT $@ -MF $(@:%o=%d) -o $@
endif
