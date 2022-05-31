/*
  ==============================================================================

    This file contains the basic framework code for a JUCE plugin processor.

  ==============================================================================
*/

#include "PluginProcessor.h"

///==============================================================================

namespace IDs
{
    static juce::String paramOutput  { "output" };
    static juce::String paramType    { "type" };
    static juce::String paramFreq    { "freq" };
    static juce::String paramGain    { "gain" };
    static juce::String paramQuality { "quality" };
    static juce::String paramPoleReal  { "pole-real" };
    static juce::String paramPoleImag  { "pole-imag" };
    static juce::String paramZeroReal  { "zero-real" };
    static juce::String paramZeroImag  { "zero-imag" };
    static juce::String paramActive  { "active" };
}

juce::StringArray filterNames =
{
    NEEDS_TRANS ("No filter"),
    NEEDS_TRANS ("High pass"),
    NEEDS_TRANS ("1st order high pass"),
    NEEDS_TRANS ("Low shelf"),
    NEEDS_TRANS ("Band pass"),
    NEEDS_TRANS ("Notch"),
    NEEDS_TRANS ("Peak"),
    NEEDS_TRANS ("High shelf"),
    NEEDS_TRANS ("1st order low pass"),
    NEEDS_TRANS ("Low pass")
};

static float maxLevel = 24.0f;

std::unique_ptr<juce::AudioProcessorParameterGroup> createParametersForFilter (const juce::String& prefix,
                                                                               const juce::String& name,
                                                                               PolesNZerosAudioProcessor::FilterType type,
                                                                               float frequency,
                                                                               float gain    = 0.0f,
                                                                               float quality = 1.0f,
                                                                               float poleReal = 0.0f,
                                                                               float poleImag = 0.0f,
                                                                               float zeroReal = 0.0f,
                                                                               float zeroImag = 0.0f,
                                                                               bool  active  = true)
{
    auto typeParameter = std::make_unique<juce::AudioParameterChoice> (prefix + IDs::paramType,
                                                                       name + ": " + TRANS ("Filter Type"),
                                                                       filterNames,
                                                                       type);

    auto actvParameter = std::make_unique<juce::AudioParameterBool> (prefix + IDs::paramActive,
                                                                     name + ": " + TRANS ("Active"),
                                                                     active,
                                                                     juce::String(),
                                                                     [](float value, int) {return value > 0.5f ? TRANS ("active") : TRANS ("bypassed");},
                                                                     [](juce::String text) {return text == TRANS ("active");});

    auto freqParameter = std::make_unique<juce::AudioParameterFloat> (prefix + IDs::paramFreq,
                                                                      name + ": " + TRANS ("Frequency"),
                                                                      foleys::Conversions::makeLogarithmicRange<float>(20.0f, 20000.0f),
                                                                      frequency,
                                                                      juce::String(),
                                                                      juce::AudioProcessorParameter::genericParameter,
                                                                      [](float value, int) { return (value < 1000) ?
                                                                        juce::String (value, 0) + " Hz" :
                                                                        juce::String (value / 1000.0) + " kHz"; },
                                                                      [](juce::String text) { return text.endsWith(" kHz") ?
                                                                        text.getFloatValue() * 1000.0f :
                                                                        text.getFloatValue(); });

    auto qltyParameter = std::make_unique<juce::AudioParameterFloat> (prefix + IDs::paramQuality,
                                                                      name + ": " + TRANS ("Quality"),
                                                                      juce::NormalisableRange<float> {0.1f, 10.0f, 0.1f, std::log (0.5f) / std::log (0.9f / 9.9f)},
                                                                      quality,
                                                                      juce::String(),
                                                                      juce::AudioProcessorParameter::genericParameter,
                                                                      [](float value, int) { return juce::String (value, 1); },
                                                                      [](const juce::String& text) { return text.getFloatValue(); });

    auto poleRealParameter = std::make_unique<juce::AudioParameterFloat> (prefix + IDs::paramPoleReal,
                                                                      name + ": " + TRANS ("Pole Real Part"),
                                                                      juce::NormalisableRange<float> {-0.999f, 0.999f}, poleReal,
                                                                      juce::String(),
                                                                      juce::AudioProcessorParameter::genericParameter,
                                                                      [](float value, int) { return juce::String (value, 1); },
                                                                      [](const juce::String& text) { return text.getFloatValue(); });

    auto poleImagParameter = std::make_unique<juce::AudioParameterFloat> (prefix + IDs::paramPoleImag,
                                                                      name + ": " + TRANS ("Pole Imag Part"),
                                                                          juce::NormalisableRange<float> {-0.999f, 0.999f}, poleImag,
                                                                      juce::String(),
                                                                      juce::AudioProcessorParameter::genericParameter,
                                                                      [](float value, int) { return juce::String (value, 1); },
                                                                      [](const juce::String& text) { return text.getFloatValue(); });

    auto zeroRealParameter = std::make_unique<juce::AudioParameterFloat> (prefix + IDs::paramZeroReal,
                                                                      name + ": " + TRANS ("Zero Real Part"),
                                                                          juce::NormalisableRange<float> {-0.999f, 0.999f}, zeroReal,
                                                                          
                                                                      juce::String(),
                                                                      juce::AudioProcessorParameter::genericParameter,
                                                                      [](float value, int) { return juce::String (value, 1); },
                                                                      [](const juce::String& text) { return text.getFloatValue(); });

    auto zeroImagParameter = std::make_unique<juce::AudioParameterFloat> (prefix + IDs::paramZeroImag,
                                                                      name + ": " + TRANS ("Zero Imag Part"),
                                                                          juce::NormalisableRange<float> {-0.999f, 0.999f}, zeroImag,
                                                                      juce::String(),
                                                                      juce::AudioProcessorParameter::genericParameter,
                                                                      [](float value, int) { return juce::String (value, 1); },
                                                                      [](const juce::String& text) { return text.getFloatValue(); });

    auto gainParameter = std::make_unique<juce::AudioParameterFloat> (prefix + IDs::paramGain,
                                                                      name + ": " + TRANS ("Gain"),
                                                                      juce::NormalisableRange<float> {-maxLevel, maxLevel, 0.1f},
                                                                      gain,
                                                                      juce::String(),
                                                                      juce::AudioProcessorParameter::genericParameter,
                                                                      [](float value, int) {return juce::String (value, 1) + " dB";},
                                                                      [](juce::String text) {return text.getFloatValue();});

    auto group = std::make_unique<juce::AudioProcessorParameterGroup> ("band" + prefix, name, "|",
                                                                       std::move (typeParameter),
                                                                       std::move (actvParameter),
                                                                       std::move (freqParameter),
                                                                       std::move (qltyParameter),
                                                                       std::move (poleRealParameter),
                                                                       std::move (poleImagParameter),
                                                                       std::move (zeroRealParameter),
                                                                       std::move (zeroImagParameter),
                                                                       std::move (gainParameter));

    return group;
}

juce::AudioProcessorValueTreeState::ParameterLayout createParameterLayout()
{
    std::vector<std::unique_ptr<juce::AudioProcessorParameterGroup>> params;

    params.push_back (createParametersForFilter ("Q1", NEEDS_TRANS ("Q1"), PolesNZerosAudioProcessor::HighPass,     40.0f));
    params.push_back (createParametersForFilter ("Q2", NEEDS_TRANS ("Q2"), PolesNZerosAudioProcessor::LowShelf,    250.0f));
    params.push_back (createParametersForFilter ("Q3", NEEDS_TRANS ("Q3"), PolesNZerosAudioProcessor::Peak,        500.0f));
    params.push_back (createParametersForFilter ("Q4", NEEDS_TRANS ("Q4"), PolesNZerosAudioProcessor::Peak,       1000.0f));
    params.push_back (createParametersForFilter ("Q5", NEEDS_TRANS ("Q5"), PolesNZerosAudioProcessor::HighShelf,  5000.0f));
    params.push_back (createParametersForFilter ("Q6", NEEDS_TRANS ("Q6"), PolesNZerosAudioProcessor::LowPass,   12000.0f));

    auto param = std::make_unique<juce::AudioParameterFloat> (IDs::paramOutput, TRANS ("Output"),
                                                              juce::NormalisableRange<float> (0.0f, 2.0f, 0.01f), 1.0f,
                                                              juce::String(),
                                                              juce::AudioProcessorParameter::genericParameter,
                                                              [](float value, int) {return juce::String (juce::Decibels::gainToDecibels(value), 1) + " dB";},
                                                              [](juce::String text) {return juce::Decibels::decibelsToGain (text.getFloatValue());});

    auto group = std::make_unique<juce::AudioProcessorParameterGroup> ("global", TRANS ("Globals"), "|", std::move (param));
    params.push_back (std::move (group));

    return { params.begin(), params.end() };
}

auto createPostUpdateLambda (foleys::MagicProcessorState& magicState, const juce::String& plotID)
{
    return [plot = magicState.getObjectWithType<foleys::MagicFilterPlot>(plotID)] (const PolesNZerosAudioProcessor::FilterAttachment& a)
    {
        if (plot != nullptr)
        {
            plot->setIIRCoefficients (a.coefficients, maxLevel);
            plot->setActive (a.isActive());
        }
    };
}

//==============================================================================
PolesNZerosAudioProcessor::PolesNZerosAudioProcessor()
#ifndef JucePlugin_PreferredChannelConfigurations
     : MagicProcessor (BusesProperties()
                     #if ! JucePlugin_IsMidiEffect
                      #if ! JucePlugin_IsSynth
                       .withInput  ("Input",  juce::AudioChannelSet::stereo(), true)
                      #endif
                       .withOutput ("Output", juce::AudioChannelSet::stereo(), true)
                     #endif
                       ),
#else
    :
#endif
    treeState (*this, nullptr, JucePlugin_Name, createParameterLayout()),
    gainAttachment (treeState, gain, IDs::paramOutput)
{
    FOLEYS_SET_SOURCE_PATH (__FILE__);
    
    // GUI MAGIC: add plots to be displayed in the GUI
    for (size_t i = 0; i < attachments.size(); ++i)
    {
        auto name = "plot" + juce::String (i + 1);
        magicState.createAndAddObject<foleys::MagicFilterPlot>(name);
        attachments.at (i)->postFilterUpdate = createPostUpdateLambda (magicState, name);
    }

    plotSum = magicState.createAndAddObject<foleys::MagicFilterPlot>("plotSum");

    // GUI MAGIC: add analyser plots
    inputAnalyser   = magicState.createAndAddObject<foleys::MagicAnalyser>("input");
    outputAnalyser  = magicState.createAndAddObject<foleys::MagicAnalyser>("output");

    // MAGIC GUI: add a meter at the output
    outputMeter = magicState.createAndAddObject<foleys::MagicLevelSource>("outputMeter");

    for (auto* parameter : getParameters())
        if (auto* p = dynamic_cast<juce::AudioProcessorParameterWithID*>(parameter))
            treeState.addParameterListener (p->paramID, this);

    // MAGIC GUI: add properties to connect visibility to
    magicState.getPropertyAsValue ("analyser:input").setValue (true);
    magicState.getPropertyAsValue ("analyser:output").setValue (true);

    inputAnalysing.attachToValue (magicState.getPropertyAsValue ("analyser:input"));
    outputAnalysing.attachToValue (magicState.getPropertyAsValue ("analyser:output"));
}

PolesNZerosAudioProcessor::~PolesNZerosAudioProcessor()
{
    for (auto* parameter : getParameters())
        if (auto* p = dynamic_cast<juce::AudioProcessorParameterWithID*>(parameter))
            treeState.removeParameterListener (p->paramID, this);
}

//==============================================================================
void PolesNZerosAudioProcessor::prepareToPlay (double sampleRate, int samplesPerBlock)
{
    const auto numChannels = getTotalNumOutputChannels();

    // GUI MAGIC: call this to set up the visualisers
    magicState.prepareToPlay (sampleRate, samplesPerBlock);
    outputMeter->setupSource (getTotalNumOutputChannels(), sampleRate, 500);

    juce::dsp::ProcessSpec spec;
    spec.sampleRate = sampleRate;
    spec.maximumBlockSize = juce::uint32 (samplesPerBlock);
    spec.numChannels = juce::uint32 (numChannels);

    filter.get<6>().setGainLinear (*treeState.getRawParameterValue (IDs::paramOutput));

    for (auto* a : attachments)
        a->setSampleRate (sampleRate);

    filter.prepare (spec);
}

void PolesNZerosAudioProcessor::releaseResources()
{
}

#ifndef JucePlugin_PreferredChannelConfigurations
bool PolesNZerosAudioProcessor::isBusesLayoutSupported (const BusesLayout& layouts) const
{
#if JucePlugin_IsMidiEffect
    ignoreUnused (layouts);
    return true;
#else

    // This checks if the input layout matches the output layout
#if ! JucePlugin_IsSynth
    if (layouts.getMainOutputChannelSet() != layouts.getMainInputChannelSet())
        return false;
#endif

    return true;
#endif
}
#endif

void PolesNZerosAudioProcessor::processBlock (juce::AudioBuffer<float>& buffer, juce::MidiBuffer& midiMessages)
{
    juce::ScopedNoDenormals noDenormals;
    ignoreUnused (midiMessages);

    filter.setBypassed<0>(attachment1.isActive() == false);
    filter.setBypassed<1>(attachment2.isActive() == false);
    filter.setBypassed<2>(attachment3.isActive() == false);
    filter.setBypassed<3>(attachment4.isActive() == false);
    filter.setBypassed<4>(attachment5.isActive() == false);
    filter.setBypassed<5>(attachment6.isActive() == false);

    filter.get<6>().setGainLinear (gain);

    // GUI MAGIC: measure before processing
    if (inputAnalysing.get())
        inputAnalyser->pushSamples (buffer);

    juce::dsp::AudioBlock<float>              ioBuffer (buffer);
    juce::dsp::ProcessContextReplacing<float> context  (ioBuffer);
    filter.process (context);

    // GUI MAGIC: measure after processing
    if (outputAnalysing.get())
        outputAnalyser->pushSamples (buffer);

    outputMeter->pushSamples (buffer);
}

//==============================================================================

PolesNZerosAudioProcessor::FilterAttachment::FilterAttachment (juce::AudioProcessorValueTreeState& stateToUse, FilterBand& filterToControl, const juce::String& prefixToUse, const juce::CriticalSection& lock)
  : state               (stateToUse),
    filter              (filterToControl),
    prefix              (prefixToUse),
    callbackLock        (lock),
    typeAttachment      (state, type,      prefix + IDs::paramType,     [&]{ updateFilter(); }),
    frequencyAttachment (state, frequency, prefix + IDs::paramFreq,     [&]{ updateFilter(); }),
    gainAttachment      (state, gain,      prefix + IDs::paramGain,     [&]{ updateFilter(); }),
    qualityAttachment   (state, quality,   prefix + IDs::paramQuality,  [&]{ updateFilter(); }),
// poleRealAttachment (state, poleReal, prefix + IDs::paramPoleReal, [&]{updateFilter(); }),
// poleImagAttachment (state, poleImag, prefix + IDs::paramPoleImag, [&]{updateFilter(); }),
// zeroRealAttachment (state, zeroReal, prefix + IDs::paramZeroReal, [&]{updateFilter(); }),
//zeroImagAttachment (state, zeroImag, prefix + IDs::paramZeroImag, [&]{updateFilter(); }),
    activeAttachment    (state, active,    prefix + IDs::paramActive,   [&]
    { if (postFilterUpdate)
        postFilterUpdate (*this);
    })
{
    updateFilter();
}

void PolesNZerosAudioProcessor::FilterAttachment::updateFilter()
{
    if (sampleRate < 20.0)
        return;

    switch (type)
    {
        case NoFilter:    coefficients = new juce::dsp::IIR::Coefficients<float> (1, 0, 1, 0); break;
        case LowPass:     coefficients = juce::dsp::IIR::Coefficients<float>::makeLowPass (sampleRate, frequency, quality); break;
        case LowPass1st:  coefficients = juce::dsp::IIR::Coefficients<float>::makeFirstOrderLowPass (sampleRate, frequency); break;
        case LowShelf:    coefficients = juce::dsp::IIR::Coefficients<float>::makeLowShelf (sampleRate, frequency, quality, juce::Decibels::decibelsToGain (gain.load())); break;
        case BandPass:    coefficients = juce::dsp::IIR::Coefficients<float>::makeBandPass (sampleRate, frequency, quality); break;
        case Notch:       coefficients = juce::dsp::IIR::Coefficients<float>::makeNotch (sampleRate, frequency, quality); break;
        case Peak:        coefficients = juce::dsp::IIR::Coefficients<float>::makePeakFilter (sampleRate, frequency, quality, juce::Decibels::decibelsToGain (gain.load())); break;
        case HighShelf:   coefficients = juce::dsp::IIR::Coefficients<float>::makeHighShelf (sampleRate, frequency, quality, juce::Decibels::decibelsToGain (gain.load())); break;
        case HighPass1st: coefficients = juce::dsp::IIR::Coefficients<float>::makeFirstOrderHighPass (sampleRate, frequency); break;
        case HighPass:    coefficients = juce::dsp::IIR::Coefficients<float>::makeHighPass (sampleRate, frequency, quality); break;
        case LastFilterID:
        default:          return;
    
    }

    {
        //calling and then deferencing the coefficients like this will get you the value
        //but why is there inly one value? Shouldn't there be like 4 or 6?
        float* test = coefficients->getRawCoefficients();
        std::cout << *test << "\n";
        
        juce::ScopedLock processLock (callbackLock);
        *filter.state = *coefficients;
        
        
    }

    if (postFilterUpdate)
        postFilterUpdate (*this);
}
void findPZ(juce::dsp::IIR::Coefficients<float>::Ptr coefficients){
    
    //first get value(s) of coefficients
    float* coeffPtr = coefficients->getRawCoefficients();
    float coeffVal = *coeffPtr;
    //somehow cast to complex...are the coefficients assumed to be real? Probably
    std::complex<float> coeffs(coeffVal, 0.0f);
    //do quadratic formula with them
    //take .real and .imag
    //return these somewhere
    //once the poles and zeros are found, set them in a pole/zero set function
    //juce::Value paramToEdit = treeState.getParameterAsValue("specCent");
    return;
    
  
    // float
}


void PolesNZerosAudioProcessor::FilterAttachment::setSampleRate (double sampleRateToUse)
{
    sampleRate = sampleRateToUse;
    updateFilter();
}

//==============================================================================

template<typename ValueType>
AttachedValue<ValueType>::AttachedValue (juce::AudioProcessorValueTreeState& stateToUse,
                                                                         std::atomic<ValueType>& valueToUse,
                                                                         const juce::String& paramToUse,
                                                                         std::function<void()> changedLambda)
  : state (stateToUse),
    value (valueToUse),
    paramID (paramToUse),
    onParameterChanged (changedLambda)
{
    // Oh uh, tried to attach to a non existing parameter
    jassert (state.getParameter (paramID) != nullptr);

    initialUpdate();
    state.addParameterListener (paramID, this);
}

template<typename ValueType>
AttachedValue<ValueType>::~AttachedValue()
{
    state.removeParameterListener (paramID, this);
}

template<typename ValueType>
void AttachedValue<ValueType>::initialUpdate()
{
    value = ValueType (*state.getRawParameterValue (paramID));
}

template<>
void AttachedValue<PolesNZerosAudioProcessor::FilterType>::initialUpdate()
{
    value = PolesNZerosAudioProcessor::FilterType (juce::roundToInt (state.getRawParameterValue (paramID)->load()));
}

template<typename ValueType>
void AttachedValue<ValueType>::parameterChanged (const juce::String&, float newValue)
{
    value = newValue;
    if (onParameterChanged)
        onParameterChanged();
}

template<>
void AttachedValue<bool>::parameterChanged (const juce::String&, float newValue)
{
    value = (newValue > 0.5f);
    if (onParameterChanged)
        onParameterChanged();
}

template<>
void AttachedValue<PolesNZerosAudioProcessor::FilterType>::parameterChanged (const juce::String&, float newValue)
{
    value = PolesNZerosAudioProcessor::FilterType (juce::roundToInt (newValue));
    if (onParameterChanged)
        onParameterChanged();
}

//==============================================================================
void PolesNZerosAudioProcessor::parameterChanged (const juce::String&, float)
{
    triggerAsyncUpdate();
}

void PolesNZerosAudioProcessor::handleAsyncUpdate()
{
    std::vector<juce::dsp::IIR::Coefficients<float>::Ptr> coefficients;
    for (auto* a : attachments)
        if (a->isActive())
            coefficients.push_back (a->coefficients);


    
    
    //this is where the coefficients are passed from the filters themselves to the actual filter plot in foleys
    plotSum->setIIRCoefficients (gain, coefficients, maxLevel);
}

//==============================================================================

juce::ValueTree PolesNZerosAudioProcessor::createGuiValueTree()
{
    juce::String text (BinaryData::magicJOS_xml, BinaryData::magicJOS_xmlSize);
    return juce::ValueTree::fromXml (text);
}

void PolesNZerosAudioProcessor::postSetStateInformation()
{
    // MAGIC GUI: let the magicState conveniently handle save and restore the state.
    //            You don't need to use that, but it also takes care of restoring the last editor size
    inputAnalysing.attachToValue (magicState.getPropertyAsValue ("analyser:input"));
    outputAnalysing.attachToValue (magicState.getPropertyAsValue ("analyser:output"));
}

//==============================================================================
const juce::String PolesNZerosAudioProcessor::getName() const
{
    return JucePlugin_Name;
}

double PolesNZerosAudioProcessor::getTailLengthSeconds() const
{
    return 0.0;
}
//==============================================================================
// This creates new instances of the plugin..
juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
    return new PolesNZerosAudioProcessor();
}
