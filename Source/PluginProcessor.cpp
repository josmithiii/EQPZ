/*
  ==============================================================================

  Extend the PGM EqualizerExample (EQA) to have a pole-zero plot.

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
    static juce::String paramActive  { "active" };
    // Parameters added beyond EqualizerExample for pole-zero plots:
    static juce::String paramPoleReal1  { "pole-real1" };
    static juce::String paramPoleImag1  { "pole-imag1" };
    static juce::String paramZeroReal1  { "zero-real1" };
    static juce::String paramZeroImag1  { "zero-imag1" };
    static juce::String paramPoleReal2  { "pole-real2" };
    static juce::String paramPoleImag2  { "pole-imag2" };
    static juce::String paramZeroReal2  { "zero-real2" };
    static juce::String paramZeroImag2  { "zero-imag2" };
    static juce::String paramPlotY {"plotY"};
    static juce::String paramPlotX {"plotX"};
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
                                                                               float poleReal1 = 0.0f,
                                                                               float poleImag1= 0.0f,
                                                                               float zeroReal1 = 0.0f,
                                                                               float zeroImag1 = 0.0f,
                                                                               float poleReal2 = 0.0f,
                                                                               float poleImag2= 0.0f,
                                                                               float zeroReal2 = 0.0f,
                                                                               float zeroImag2 = 0.0f,
                                                                               float plotY = 0.0f,
                                                                               float plotX = 0.0f,
                                                                               bool  active  = true)
{
    auto typeParameter = std::make_unique<juce::AudioParameterChoice> (juce::ParameterID (prefix + IDs::paramType, 1),
                                                                       name + ": " + TRANS ("Filter Type"),
                                                                       filterNames,
                                                                       type);

    auto actvParameter = std::make_unique<juce::AudioParameterBool> (juce::ParameterID (prefix + IDs::paramActive, 1),
                                                                     name + ": " + TRANS ("Active"),
                                                                     active,
                                                                     juce::String(),
                                                                     [](float value, int) {return value > 0.5f ? TRANS ("active") : TRANS ("bypassed");},
                                                                     [](juce::String text) {return text == TRANS ("active");});

    auto freqParameter = std::make_unique<juce::AudioParameterFloat> (juce::ParameterID (prefix + IDs::paramFreq, 1),
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

    auto qltyParameter = std::make_unique<juce::AudioParameterFloat> (juce::ParameterID (prefix + IDs::paramQuality, 1),
                                                                      name + ": " + TRANS ("Quality"),
                                                                      juce::NormalisableRange<float> {0.1f, 10.0f, 0.1f, std::log (0.5f) / std::log (0.9f / 9.9f)},
                                                                      quality,
                                                                      juce::String(),
                                                                      juce::AudioProcessorParameter::genericParameter,
                                                                      [](float value, int) { return juce::String (value, 1); },
                                                                      [](const juce::String& text) { return text.getFloatValue(); });

    auto poleRealParameter1 = std::make_unique<juce::AudioParameterFloat> (juce::ParameterID (prefix + IDs::paramPoleReal1, 1),
                                                                      name + ": " + TRANS ("Pole Real Part1"),
                                                                      juce::NormalisableRange<float> {-0.999f, 0.999f}, poleReal1,
                                                                      juce::String(),
                                                                      juce::AudioProcessorParameter::genericParameter,
                                                                      [](float value, int) { return juce::String (value, 1); },
                                                                      [](const juce::String& text) { return text.getFloatValue(); });

    auto poleImagParameter1 = std::make_unique<juce::AudioParameterFloat> (juce::ParameterID (prefix + IDs::paramPoleImag1, 1),
                                                                      name + ": " + TRANS ("Pole Imag Part1"),
                                                                          juce::NormalisableRange<float> {-0.999f, 0.999f}, poleImag1,
                                                                      juce::String(),
                                                                      juce::AudioProcessorParameter::genericParameter,
                                                                      [](float value, int) { return juce::String (value, 1); },
                                                                      [](const juce::String& text) { return text.getFloatValue(); });

    auto zeroRealParameter1 = std::make_unique<juce::AudioParameterFloat> (juce::ParameterID (prefix + IDs::paramZeroReal1, 1),
                                                                      name + ": " + TRANS ("Zero Real Part1"),
                                                                          juce::NormalisableRange<float> {-0.999f, 0.999f}, zeroReal1,
                                                                      juce::String(),
                                                                      juce::AudioProcessorParameter::genericParameter,
                                                                      [](float value, int) { return juce::String (value, 1); },
                                                                      [](const juce::String& text) { return text.getFloatValue(); });

    auto zeroImagParameter1 = std::make_unique<juce::AudioParameterFloat> (juce::ParameterID (prefix + IDs::paramZeroImag1, 1),
                                                                      name + ": " + TRANS ("Zero Imag Part1"),
                                                                          juce::NormalisableRange<float> {-0.999f, 0.999f}, zeroImag1,
                                                                      juce::String(),
                                                                      juce::AudioProcessorParameter::genericParameter,
                                                                      [](float value, int) { return juce::String (value, 1); },
                                                                      [](const juce::String& text) { return text.getFloatValue(); });

    auto poleRealParameter2 = std::make_unique<juce::AudioParameterFloat> (juce::ParameterID (prefix + IDs::paramPoleReal2, 1),
                                                                      name + ": " + TRANS ("Pole Real Part2"),
                                                                      juce::NormalisableRange<float> {-0.999f, 0.999f}, poleReal2,
                                                                      juce::String(),
                                                                      juce::AudioProcessorParameter::genericParameter,
                                                                      [](float value, int) { return juce::String (value, 1); },
                                                                      [](const juce::String& text) { return text.getFloatValue(); });

    auto poleImagParameter2 = std::make_unique<juce::AudioParameterFloat> (juce::ParameterID (prefix + IDs::paramPoleImag2, 1),
                                                                      name + ": " + TRANS ("Pole Imag Part2"),
                                                                          juce::NormalisableRange<float> {-0.999f, 0.999f}, poleImag2,
                                                                      juce::String(),
                                                                      juce::AudioProcessorParameter::genericParameter,
                                                                      [](float value, int) { return juce::String (value, 1); },
                                                                      [](const juce::String& text) { return text.getFloatValue(); });

    auto zeroRealParameter2 = std::make_unique<juce::AudioParameterFloat> (juce::ParameterID (prefix + IDs::paramZeroReal2, 1),
                                                                      name + ": " + TRANS ("Zero Real Part1"),
                                                                          juce::NormalisableRange<float> {-0.999f, 0.999f}, zeroReal2,

                                                                      juce::String(),
                                                                      juce::AudioProcessorParameter::genericParameter,
                                                                      [](float value, int) { return juce::String (value, 1); },
                                                                      [](const juce::String& text) { return text.getFloatValue(); });

    auto zeroImagParameter2 = std::make_unique<juce::AudioParameterFloat> (juce::ParameterID (prefix + IDs::paramZeroImag2, 1),
                                                                      name + ": " + TRANS ("Zero Imag Part2"),
                                                                          juce::NormalisableRange<float> {-0.999f, 0.999f}, zeroImag2,
                                                                      juce::String(),
                                                                      juce::AudioProcessorParameter::genericParameter,
                                                                           [](float value, int) { return juce::String (value, 1); },
                                                                           [](const juce::String& text) { return text.getFloatValue(); });

    auto plotYParameter = std::make_unique<juce::AudioParameterFloat> (juce::ParameterID (prefix + IDs::paramPlotY, 1),
                                                                      name + ": " + TRANS ("Plot Param Y"),
                                                                          juce::NormalisableRange<float> {-0.999f, 0.999f}, plotY,
                                                                      juce::String(),
                                                                      juce::AudioProcessorParameter::genericParameter,
                                                                      [](float value, int) { return juce::String (value, 1); },
                                                                      [](const juce::String& text) { return text.getFloatValue(); });
    auto plotXParameter = std::make_unique<juce::AudioParameterFloat> (juce::ParameterID (prefix + IDs::paramPlotX, 1),
                                                                      name + ": " + TRANS ("Plot Param X"),
                                                                          juce::NormalisableRange<float> {-0.999f, 0.999f}, plotX,
                                                                      juce::String(),
                                                                      juce::AudioProcessorParameter::genericParameter,
                                                                      [](float value, int) { return juce::String (value, 1); },
                                                                      [](const juce::String& text) { return text.getFloatValue(); });

    auto gainParameter = std::make_unique<juce::AudioParameterFloat> (juce::ParameterID (prefix + IDs::paramGain, 1),
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
                                                                       std::move (poleRealParameter1),
                                                                       std::move (poleImagParameter1),
                                                                       std::move (zeroRealParameter1),
                                                                       std::move (zeroImagParameter1),
                                                                       std::move (poleRealParameter2),
                                                                       std::move (poleImagParameter2),
                                                                       std::move (zeroRealParameter2),
                                                                       std::move (zeroImagParameter2),
                                                                       std::move (plotYParameter),
                                                                       std::move (plotXParameter),
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

    auto param = std::make_unique<juce::AudioParameterFloat> (juce::ParameterID (IDs::paramOutput, 1),
                                                              TRANS ("Output"),
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

    // GUI MAGIC: input signal before processing
    if (inputAnalysing.get())
        inputAnalyser->pushSamples (buffer);

    juce::dsp::AudioBlock<float>              ioBuffer (buffer);
    juce::dsp::ProcessContextReplacing<float> context  (ioBuffer);
    filter.process (context);

    // GUI MAGIC: output signal after processing
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
    activeAttachment    (state, active,    prefix + IDs::paramActive,   [&]
    { if (postFilterUpdate)
        postFilterUpdate (*this);
    })
{
    updateFilter();
}

juce::String PolesNZerosAudioProcessor::FilterAttachment::getPrefix()
{
    return prefix;
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
         /* added: */ float filtOrder = coefficients->getFilterOrder();
        // can be 0 if "No Filter": jassert (filtOrder == 2);
        //juce::Array<float> testArr = coefficients->coefficients;
        //std::cout <<"test array abilities"<<testArr[5]<<std::endl;
        //std::cout <<"test array size "<< testArr.size() <<std::endl;

        juce::ScopedLock processLock (callbackLock);
        *filter.state = *coefficients;
    }

    if (postFilterUpdate)
        postFilterUpdate (*this);
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

#if 0
void PolesNZerosAudioProcessor::handleAsyncUpdateOriginal()
// Original version from EqualizerExample for reference.
// Up to here we've only really added new parameters.
// GUI elements listen to their own parameters so shared params move both knobs and XY sliders.
// All we need to do is recompute the magnitude frequency-response (MFR) displays.
{
    std::vector<juce::dsp::IIR::Coefficients<float>::Ptr> coefficients; // for MFR displays
    for (auto* a : attachments)
        if (a->isActive())
            coefficients.push_back (a->coefficients); // each band has its own juce::dsp::IIR::Coefficients<float>(b0,b1,[b2],a0,a1,[a2])

    plotSum->setIIRCoefficients (gain, coefficients, maxLevel); // send the array of coefficients to MagicFilterPlot
}
#endif

void PolesNZerosAudioProcessor::handleAsyncUpdate()
// Scheduled by triggerAsyncUpdate() which was called by parameterChanged()
// We listen to all parameters here.
// Our job is to update all filter representations when anything changes:
//   poles and zeros display (XY sliders)
//   frequency-response display
//   frequency-response XY sliders
// I think parameter-knobs and XY-sliders controlling the same parameter
//   stay in sync because they separately listen to their parameters.
{

    std::vector<juce::dsp::IIR::Coefficients<float>::Ptr> coefficients; // for frequency-response display
    int count = 1;
    juce::dsp::IIR::Coefficients<float>::Ptr holder;

    for (auto* a : attachments){
        if (/* JOS: We crashed here: 1 || */
            a->isActive() ) { // FIXME: a->isActive() is always FALSE due to some BUG (in PGM I think)

            juce::String prefixMaybe = a->getPrefix();
            std::cout<<"prefix is "<< prefixMaybe << std::endl;

            juce::Value ParamToEdit_pr1 = treeState.getParameterAsValue(prefixMaybe + "pole-real1");
            juce::Value ParamToEdit_pi1 = treeState.getParameterAsValue(prefixMaybe + "pole-imag1");
            juce::Value ParamToEdit_pr2 = treeState.getParameterAsValue(prefixMaybe + "pole-real2");
            juce::Value ParamToEdit_pi2 = treeState.getParameterAsValue(prefixMaybe + "pole-imag2");
            juce::Value ParamToEdit_zr1 = treeState.getParameterAsValue(prefixMaybe + "zero-real1");
            juce::Value ParamToEdit_zi1 = treeState.getParameterAsValue(prefixMaybe + "zero-imag1");
            juce::Value ParamToEdit_zr2 = treeState.getParameterAsValue(prefixMaybe + "zero-real2");
            juce::Value ParamToEdit_zi2 = treeState.getParameterAsValue(prefixMaybe + "zero-imag2");
            juce::Value ParamToEdit_plotY = treeState.getParameterAsValue(prefixMaybe +"plotY");
            juce::Value ParamToEdit_plotX = treeState.getParameterAsValue(prefixMaybe +"plotX");

            //in here do the bit swapping

            //first get values of coefficients
            holder = a->coefficients;
            juce::Array<float> coeffVals = holder->coefficients;
            //std::cout <<"TEST"<<coeffVals<<std::endl;
            //cast them to complex numbers for safekeeping ( assuming that coefficients are real...which seems reasonable)
            //zeros (numerator) first
            std::complex<float> a_zero(coeffVals[0], 0.0f);
            std::complex<float> b_zero(coeffVals[1], 0.0f);
            std::complex<float> c_zero(coeffVals[2], 0.0f);
            //poles (denom) second
            std::complex<float> a_pole(coeffVals[3], 0.0f);
            std::complex<float> b_pole(coeffVals[4], 0.0f);
            std::complex<float> c_pole(0.0f, 0.0f); //array only has length 5 for second order filters

            //find roots
            static const float MIN_POLE { 1.0e-7f };
            if (abs(a_pole) < MIN_POLE)
                a_pole = MIN_POLE;
            std::complex<float> zero1 = ((-b_zero) + sqrt((pow(b_zero,2.0f) - (4.0f*a_zero*c_zero))))/(2.0f*a_zero);
            std::complex<float> zero2 = ((-b_zero) - sqrt((pow(b_zero,2.0f) - (4.0f*a_zero*c_zero))))/(2.0f*a_zero);
            std::complex<float> pole1 = ((-b_pole) + sqrt((pow(b_pole,2.0f) - (4.0f*a_pole*c_pole))))/(2.0f*a_pole);
            std::complex<float> pole2 = ((-b_pole) - sqrt((pow(b_pole,2.0f) - (4.0f*a_pole*c_pole))))/(2.0f*a_pole);

            std::cout <<"filter is "<<count<<std::endl;
            std::cout <<"zero 1 is "<<zero1<<std::endl;
            std::cout <<"zero 2 is "<<zero2<<std::endl;
            std::cout <<"pole 1 is "<<pole1<<std::endl;
            std::cout <<"pole 2 is "<<pole2<<std::endl;

            count = count + 1;

            ParamToEdit_pr1 = pole1.real();
            ParamToEdit_pi1 = pole1.imag();
            ParamToEdit_pr2 = pole2.real();
            ParamToEdit_pi2 = pole2.imag();
            ParamToEdit_zr1 = zero1.real();
            ParamToEdit_zi1 = zero1.imag();
            ParamToEdit_zr2 = zero2.real();
            ParamToEdit_zi2 = zero2.imag();

            ParamToEdit_plotY = 0.0f; //unnecessary to have it repeat for each loop but should work
            ParamToEdit_plotX = 0.0f;

            coefficients.push_back (a->coefficients);
    //set the coefficients or steal them depending
    //be careful with the listener
        }

    }



    //this is where the coefficients are passed from the filters themselves to the actual filter plot in foleys
    plotSum->setIIRCoefficients (gain, coefficients, maxLevel);
}

//==============================================================================

juce::ValueTree PolesNZerosAudioProcessor::createGuiValueTree()
{
    juce::String text (BinaryData::magic_xml, BinaryData::magic_xmlSize);
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
