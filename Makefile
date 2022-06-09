pi preInstall: clean
	togglePGM JOS
	open ~/JUCE/Projucer.app GeoM.jucer

clean:
	-/bin/rm -rf Builds JuceLibraryCode
	-/bin/rm -rf Source/magic.sav*
