within Flex;
model X3AWithRadiantFloor "Example model showing a use of X3A"
  extends Modelica.Icons.Example;

  package Air = Buildings.Media.Air "Air model used in the example model";
  package Water = Buildings.Media.Water
    "Water model used in the radiant slab loop";

  parameter Real latitude = 0.656593 "Latitude";
  parameter Real longitude = -2.13628 "Longitude";
  parameter Modelica.SIunits.Time timZon = -28800 "Time zone";

  Modelica.Blocks.Interfaces.RealInput TNeiRoo(unit="K")
    "Temperature of neighboring rooms"
    annotation (Placement(transformation(extent={{238,-130},{198,-90}})));
  Modelica.Blocks.Interfaces.RealInput TDryBulOut(unit="K")
    "Outside dry bulb temperature"
    annotation (Placement(transformation(extent={{-340,170},{-300,210}})));
  Modelica.Blocks.Interfaces.RealInput HDifHor(unit="W/m2")
    "Input diffuse horizontal radiation"
    annotation (Placement(transformation(extent={{-340,120},{-300,160}})));
  Modelica.Blocks.Interfaces.RealInput HDirNor(unit="W/m2")
    "Direct normal irradiation"
    annotation (Placement(transformation(extent={{-340,80},{-300,120}})));
  Modelica.Blocks.Interfaces.RealInput qGai_flow[3](each unit="W/m2")
    "Radiant, convective and latent heat input into room (positive if heat gain)"
    annotation (Placement(transformation(extent={{-340,20},{-300,60}})));
  Modelica.Blocks.Interfaces.RealInput mHVAC_flow(unit="kg/s")
    "HVAC mass flow rate"
    annotation (Placement(transformation(extent={{-340,-40},{-300,0}})));
  Modelica.Blocks.Interfaces.RealInput THVACSup "HVAC supply air temperature"
    annotation (Placement(transformation(extent={{-340,-80},{-300,-40}})));
  Modelica.Blocks.Interfaces.RealInput mSlab_flow[4](each unit="kg/s")
    "Mass flow rate of each radiant slab"
    annotation (Placement(transformation(extent={{-340,-120},{-300,-80}})));
  Modelica.Blocks.Interfaces.RealInput TSlabIn(unit="K")
    "Inlet water temperature into the four slabs (all the same)"
    annotation (Placement(transformation(extent={{-342,-170},{-302,-130}})));

  Buildings.Rooms.FLEXLAB.Rooms.X3A.TestCell X3A(
    nPorts=2,
    redeclare package Medium = Air,
    linearizeRadiation=false,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    extConMod=Buildings.HeatTransfer.Types.ExteriorConvection.Fixed)
              annotation (Placement(transformation(extent={{-50,38},{-10,78}})));
  Buildings.Fluid.Sources.MassFlowSource_T airIn(
    use_m_flow_in=true,
    use_T_in=true,
    redeclare package Medium = Air,
    nPorts=1) "Inlet air conditions (from AHU) for X3A"
    annotation (Placement(transformation(extent={{-100,50},{-80,70}})));
  Buildings.Fluid.Sources.Boundary_pT
    airOut(nPorts=1, redeclare package Medium = Air) "Air outlet for X3A"
    annotation (Placement(transformation(extent={{-98,24},{-78,44}})));
  Buildings.Fluid.HeatExchangers.RadiantSlabs.SingleCircuitSlab sla4A1(
    sysTyp=Buildings.Fluid.HeatExchangers.RadiantSlabs.Types.SystemType.Floor,
    iLayPip=1,
    redeclare package Medium = Water,
    pipe=pipe,
    layers=slaCon,
    m_flow_nominal=0.504,
    A=6.645*3.09,
    length=32.92,
    disPip=sla4A1.A/sla4A1.length,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
    "Radiant slab serving the north side of cell X3A. Name is taken from drawing M3.02"
    annotation (Placement(transformation(extent={{-48,-136},{-28,-116}})));

  Buildings.Fluid.Sources.MassFlowSource_T watIn4A1(
    nPorts=1,
    use_m_flow_in=true,
    use_T_in=true,
    redeclare package Medium = Water)
    "Inlet water conditions (from central plant)"
    annotation (Placement(transformation(extent={{-112,-136},{-92,-116}})));
  Buildings.Fluid.Sources.Boundary_pT watOut4A1(nPorts=1, redeclare package
      Medium = Water) "Water outlet"
                 annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={0,-126})));
  Buildings.HeatTransfer.Sources.PrescribedTemperature preT
    "Temperature of the ground"
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-34,-170})));
  Modelica.Blocks.Sources.CombiTimeTable TGro(
    table=[0,288.15; 86400,288.15], tableOnFile=false)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-34,-198})));
  parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic
    slaCon(nLay=3, material={
      Buildings.HeatTransfer.Data.Solids.Generic(
        x=0.1524,
        k=1.13,
        c=1000,
        d=1400,
        nSta=5),
      Buildings.HeatTransfer.Data.Solids.Generic(
        x=0.127,
        k=0.036,
        c=1200,
        d=40),
      Buildings.HeatTransfer.Data.Solids.Generic(
        x=0.2,
        k=1.8,
        c=1100,
        d=2400)}) "Construction of the slab"
    annotation (Placement(transformation(extent={{-136,-208},{-116,-188}})));
  parameter Buildings.Fluid.Data.Pipes.PEX_RADTEST pipe(dIn=0.015875, dOut=0.01905)
    annotation (Placement(transformation(extent={{-136,-186},{-116,-166}})));

  Buildings.HeatTransfer.Sources.PrescribedTemperature preT2[5]   annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={110,-110})));

  Buildings.Fluid.Sources.MassFlowSource_T watIn4A2(
    nPorts=1,
    use_m_flow_in=true,
    use_T_in=true,
    redeclare package Medium = Water)
    "Inlet water conditions (from central plant)"
    annotation (Placement(transformation(extent={{-146,-100},{-126,-80}})));
  Buildings.Fluid.HeatExchangers.RadiantSlabs.SingleCircuitSlab sla4A2(
    sysTyp=Buildings.Fluid.HeatExchangers.RadiantSlabs.Types.SystemType.Floor,
    iLayPip=1,
    redeclare package Medium = Water,
    pipe=pipe,
    layers=slaCon,
    m_flow_nominal=0.504,
    A=6.645*1.51,
    disPip=sla4A2.A/sla4A2.length,
    length=45.11,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
    "Radiant slab serving the north-central section of cell X3A. Name is taken from drawing M3.02"
    annotation (Placement(transformation(extent={{-92,-100},{-72,-80}})));

  Buildings.Fluid.Sources.Boundary_pT watOut4A2(nPorts=1, redeclare package
      Medium = Water) "Water outlet"
                 annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-52,-90})));
  Buildings.Fluid.Sources.MassFlowSource_T watIn4A3(
    nPorts=1,
    use_m_flow_in=true,
    use_T_in=true,
    redeclare package Medium = Water)
    "Inlet water conditions (from central plant)"
    annotation (Placement(transformation(extent={{-178,-62},{-158,-42}})));
  Buildings.Fluid.HeatExchangers.RadiantSlabs.SingleCircuitSlab sla4A3(
    sysTyp=Buildings.Fluid.HeatExchangers.RadiantSlabs.Types.SystemType.Floor,
    iLayPip=1,
    redeclare package Medium = Water,
    pipe=pipe,
    layers=slaCon,
    m_flow_nominal=0.504,
    A=6.645*0.91,
    disPip=sla4A3.A/sla4A3.length,
    length=42.98,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
    "Radiant slab serving the south-central section of cell X3A. Name is taken from drawing M3.02"
    annotation (Placement(transformation(extent={{-132,-62},{-112,-42}})));

  Buildings.Fluid.Sources.Boundary_pT watOut4A3(nPorts=1, redeclare package
      Medium = Water) "Water outlet"
                 annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-92,-52})));
  Buildings.Fluid.Sources.MassFlowSource_T watIn4A4(
    nPorts=1,
    use_m_flow_in=true,
    use_T_in=true,
    redeclare package Medium = Water)
    "Inlet water conditions (from central plant)"
    annotation (Placement(transformation(extent={{-192,-16},{-172,4}})));
  Buildings.Fluid.HeatExchangers.RadiantSlabs.SingleCircuitSlab sla4A4(
    sysTyp=Buildings.Fluid.HeatExchangers.RadiantSlabs.Types.SystemType.Floor,
    iLayPip=1,
    redeclare package Medium = Water,
    pipe=pipe,
    layers=slaCon,
    m_flow_nominal=0.504,
    A=6.645*3.65,
    disPip=sla4A4.A/sla4A4.length,
    length=50.9,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
    "Radiant slab serving the south section of cell X3A. Name is taken from drawing M3.02"
    annotation (Placement(transformation(extent={{-162,-16},{-142,4}})));

  Buildings.Fluid.Sources.Boundary_pT watOut4A4(nPorts=1, redeclare package
      Medium = Water) "Water outlet"
                 annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-122,-6})));
  Modelica.Blocks.Routing.Replicator TNei(nout=5)
    "Temperature of neighboring rooms"
    annotation (Placement(transformation(extent={{160,-120},{140,-100}})));

  Modelica.Blocks.Routing.DeMultiplex4 deMultiplex4_1
    "Mass flow rate demultiplex"
    annotation (Placement(transformation(extent={{-280,-110},{-260,-90}})));

  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor TRoo
    "Room air temperature sensor"
    annotation (Placement(transformation(extent={{12,50},{32,70}})));
  Modelica.Blocks.Interfaces.RealOutput TRooAir(unit="K")
    "Room air temperature"
    annotation (Placement(transformation(extent={{200,50},{220,70}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor TSla[4]
    "Slab surface temperature"
    annotation (Placement(transformation(extent={{20,-30},{40,-10}})));
  Modelica.Blocks.Interfaces.RealOutput TSlabSur[4](unit="K")
    "Slab surface temperature"
    annotation (Placement(transformation(extent={{200,-30},{220,-10}})));
protected
   Buildings.BoundaryConditions.SolarGeometry.BaseClasses.Declination
                                         decAng "Declination angle"
    annotation (Placement(transformation(extent={{-90,280},{-70,300}})));
   Buildings.BoundaryConditions.SolarGeometry.BaseClasses.SolarHourAngle
    solHouAng
    annotation (Placement(transformation(extent={{-90,250},{-70,270}})));
   Buildings.BoundaryConditions.SolarGeometry.BaseClasses.ZenithAngle
                                         zenAng(final lat=latitude)
    "Zenith angle"
    annotation (Placement(transformation(extent={{-30,274},{-10,294}})));
  Buildings.BoundaryConditions.SolarGeometry.BaseClasses.AltitudeAngle
                                          altAng "Solar altitude angle"
    annotation (Placement(transformation(extent={{20,218},{40,238}})));
  Buildings.BoundaryConditions.WeatherData.BaseClasses.SolarTime
                        solTim "Solar time"
    annotation (Placement(transformation(extent={{-30,360},{-10,380}})));
  Buildings.BoundaryConditions.WeatherData.BaseClasses.LocalCivilTime locTim(
                     final timZon=timZon, final lon=longitude)
    "Local civil time"
    annotation (Placement(transformation(extent={{-66,340},{-46,360}})));
  Buildings.Utilities.Time.ModelTime
                           modTim "Model time"
    annotation (Placement(transformation(extent={{-140,400},{-120,420}})));
public
  Buildings.BoundaryConditions.WeatherData.Bus weaBus annotation (Placement(
        transformation(extent={{-154,124},{-114,164}}), iconTransformation(
          extent={{-800,96},{-780,116}})));
protected
  Buildings.BoundaryConditions.WeatherData.BaseClasses.EquationOfTime
                             eqnTim "Equation of time"
    annotation (Placement(transformation(extent={{-70,380},{-50,400}})));
public
  Modelica.Blocks.Sources.Constant zer(k=0)
    annotation (Placement(transformation(extent={{-200,160},{-180,180}})));
  Modelica.Blocks.Sources.Constant lat(k=latitude) "Latitude"
    annotation (Placement(transformation(extent={{-200,112},{-180,132}})));
  Modelica.Blocks.Sources.Constant lon(k=longitude) "Longitude"
    annotation (Placement(transformation(extent={{-200,74},{-180,94}})));
equation
  connect(airIn.ports[1], X3A.ports[1]) annotation (Line(
      points={{-80,60},{-72,60},{-72,46},{-45,46}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(airOut.ports[1], X3A.ports[2]) annotation (Line(
      points={{-78,34},{-72,34},{-72,50},{-45,50}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(sla4A1.surf_b, preT.port)               annotation (Line(
      points={{-34,-136},{-34,-160}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(watIn4A1.ports[1], sla4A1.port_a)
                                         annotation (Line(
      points={{-92,-126},{-48,-126}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(sla4A1.port_b, watOut4A1.ports[1])
                                          annotation (Line(
      points={{-28,-126},{-10,-126}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TGro.y[1],preT. T) annotation (Line(
      points={{-34,-187},{-34,-182}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(sla4A1.surf_a, X3A.surf_surBou[1]) annotation (Line(
      points={{-34,-116},{-34,43.25},{-33.8,43.25}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(watIn4A2.ports[1], sla4A2.port_a) annotation (Line(
      points={{-126,-90},{-92,-90}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(sla4A2.port_b, watOut4A2.ports[1]) annotation (Line(
      points={{-72,-90},{-62,-90}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(sla4A2.surf_a, X3A.surf_surBou[2]) annotation (Line(
      points={{-78,-80},{-78,-48},{-33.8,-48},{-33.8,43.75}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(preT.port, sla4A2.surf_b) annotation (Line(
      points={{-34,-160},{-78,-160},{-78,-100}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(watIn4A3.ports[1], sla4A3.port_a) annotation (Line(
      points={{-158,-52},{-132,-52}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(sla4A3.port_b, watOut4A3.ports[1]) annotation (Line(
      points={{-112,-52},{-102,-52}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(sla4A3.surf_a, X3A.surf_surBou[3]) annotation (Line(
      points={{-118,-42},{-118,-24},{-33.8,-24},{-33.8,44.25}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(preT.port, sla4A3.surf_b) annotation (Line(
      points={{-34,-160},{-78,-160},{-78,-106},{-118,-106},{-118,-62}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(watIn4A4.ports[1], sla4A4.port_a) annotation (Line(
      points={{-172,-6},{-162,-6}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(sla4A4.port_b, watOut4A4.ports[1]) annotation (Line(
      points={{-142,-6},{-132,-6}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(preT.port, sla4A4.surf_b) annotation (Line(
      points={{-34,-160},{-78,-160},{-78,-106},{-118,-106},{-118,-72},{-148,-72},
          {-148,-16}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(sla4A4.surf_a, X3A.surf_surBou[4]) annotation (Line(
      points={{-148,4},{-148,14},{-33.8,14},{-33.8,44.75}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(preT2.port, X3A.surf_conBou) annotation (Line(points={{100,-110},{48,-110},
          {-24,-110},{-24,42}},       color={191,0,0}));
  connect(TNei.y, preT2.T)
    annotation (Line(points={{139,-110},{122,-110}}, color={0,0,127}));
  connect(TNei.u, TNeiRoo)
    annotation (Line(points={{162,-110},{218,-110}}, color={0,0,127}));
  connect(X3A.qGai_flow, qGai_flow) annotation (Line(points={{-52,66},{-60,66},{
          -60,64},{-70,64},{-70,100},{-250,100},{-250,40},{-320,40}}, color={0,0,
          127}));
  connect(airIn.m_flow_in, mHVAC_flow) annotation (Line(points={{-100,68},{-240,
          68},{-240,68},{-240,-20},{-320,-20},{-320,-20}}, color={0,0,127}));
  connect(airIn.T_in, THVACSup) annotation (Line(points={{-102,64},{-148,64},{-148,
          62},{-236,62},{-236,-60},{-320,-60}}, color={0,0,127}));
  connect(deMultiplex4_1.u, mSlab_flow)
    annotation (Line(points={{-282,-100},{-320,-100}}, color={0,0,127}));
  connect(deMultiplex4_1.y1[1], watIn4A1.m_flow_in) annotation (Line(points={{-259,
          -91},{-220,-91},{-220,-118},{-112,-118}}, color={0,0,127}));
  connect(deMultiplex4_1.y2[1], watIn4A2.m_flow_in) annotation (Line(points={{-259,
          -97},{-186,-97},{-186,-82},{-146,-82}}, color={0,0,127}));
  connect(deMultiplex4_1.y3[1], watIn4A3.m_flow_in) annotation (Line(points={{-259,
          -103},{-200,-103},{-200,-44},{-178,-44}}, color={0,0,127}));
  connect(deMultiplex4_1.y4[1], watIn4A4.m_flow_in) annotation (Line(points={{-259,
          -109},{-259,-108.5},{-192,-108.5},{-192,2}}, color={0,0,127}));
  connect(TSlabIn, watIn4A4.T_in) annotation (Line(points={{-322,-150},{-262,-150},
          {-262,-150},{-210,-150},{-210,-2},{-194,-2}}, color={0,0,127}));
  connect(TSlabIn, watIn4A3.T_in) annotation (Line(points={{-322,-150},{-264,-150},
          {-210,-150},{-210,-48},{-180,-48}}, color={0,0,127}));
  connect(TSlabIn, watIn4A2.T_in) annotation (Line(points={{-322,-150},{-266,-150},
          {-210,-150},{-210,-86},{-148,-86}}, color={0,0,127}));
  connect(TSlabIn, watIn4A1.T_in) annotation (Line(points={{-322,-150},{-250,-150},
          {-250,-150},{-210,-150},{-210,-122},{-114,-122}}, color={0,0,127}));
  connect(X3A.heaPorAir, TRoo.port) annotation (Line(points={{-31,58},{-10,58},
          {-10,60},{12,60}}, color={191,0,0}));
  connect(TRoo.T, TRooAir)
    annotation (Line(points={{32,60},{210,60}}, color={0,0,127}));
  connect(TSla[1].port, sla4A1.surf_a) annotation (Line(points={{20,-20},{20,
          -20},{-34,-20},{-34,-116}}, color={191,0,0}));
  connect(TSla[2].port, sla4A2.surf_a) annotation (Line(points={{20,-20},{-34,
          -20},{-34,-48},{-78,-48},{-78,-80}}, color={191,0,0}));
  connect(TSla[3].port, sla4A3.surf_a) annotation (Line(points={{20,-20},{-34,
          -20},{-34,-24},{-116,-24},{-116,-42},{-118,-42}}, color={191,0,0}));
  connect(TSla[4].port, sla4A4.surf_a) annotation (Line(points={{20,-20},{-8,
          -20},{-34,-20},{-34,14},{-148,14},{-148,4}}, color={191,0,0}));
  connect(TSla.T, TSlabSur)
    annotation (Line(points={{40,-20},{210,-20}}, color={0,0,127}));
  connect(decAng.decAng,zenAng. decAng)
                                  annotation (Line(
      points={{-69,290},{-32,290},{-32,289.4}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(solHouAng.solHouAng,zenAng. solHouAng)                                              annotation (Line(
      points={{-69,260},{-50,260},{-50,279.2},{-32,279.2}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(zenAng.zen,altAng. zen) annotation (Line(
      points={{-9,284},{10,284},{10,228},{18,228}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(eqnTim.eqnTim,solTim. equTim) annotation (Line(
      points={{-49,390},{-38,390},{-38,376},{-32,376}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(locTim.locTim,solTim. locTim) annotation (Line(
      points={{-45,350},{-38,350},{-38,364.6},{-32,364.6}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(modTim.y,locTim. cloTim) annotation (Line(
      points={{-119,410},{-100,410},{-100,350},{-68,350}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(modTim.y, eqnTim.nDay) annotation (Line(
      points={{-119,410},{-100,410},{-100,390},{-72,390}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(solHouAng.solTim,solTim. solTim) annotation (Line(
      points={{-92,260},{-104,260},{-104,328},{30,328},{30,370},{-9,370}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(decAng.nDay,modTim. y) annotation (Line(
      points={{-92,290},{-100,290},{-100,320},{50,320},{50,410},{-119,410}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(weaBus, X3A.weaBus) annotation (Line(
      points={{-134,144},{-84,144},{-12.1,144},{-12.1,75.9}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  connect(TDryBulOut, weaBus.TDryBul) annotation (Line(points={{-320,190},{-134,
          190},{-134,144}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}}));
  connect(solTim.solTim, weaBus.solTim) annotation (Line(points={{-9,370},{8,370},
          {8,144},{-134,144}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}}));
  connect(modTim.y, weaBus.cloTim) annotation (Line(points={{-119,410},{-100,
          410},{-100,408},{-100,350},{-134,350},{-134,144}},
                                                        color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}}));
  connect(HDirNor, weaBus.HDirNor) annotation (Line(points={{-320,100},{-260,100},
          {-260,144},{-134,144}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}}));
  connect(HDifHor, weaBus.HDifHor) annotation (Line(points={{-320,140},{-228,140},
          {-228,144},{-134,144}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}}));
  connect(altAng.alt, weaBus.solAlt) annotation (Line(points={{41,228},{52,228},
          {52,146},{-134,146},{-134,144}},          color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}}));
  connect(decAng.decAng, weaBus.solDec) annotation (Line(points={{-69,290},{-56,
          290},{-56,144},{-134,144}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}}));
  connect(zer.y, weaBus.winDir) annotation (Line(points={{-179,170},{-166,170},{
          -166,172},{-134,172},{-134,144}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}}));
  connect(zer.y, weaBus.winSpe) annotation (Line(points={{-179,170},{-158,170},{
          -158,172},{-134,172},{-134,144}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}}));
  connect(solHouAng.solHouAng, weaBus.solHouAng) annotation (Line(points={{-69,260},
          {-62,260},{-62,144},{-134,144}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}}));
  connect(lat.y, weaBus.lat) annotation (Line(points={{-179,122},{-160,122},{-134,
          122},{-134,144}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}}));
  connect(lon.y, weaBus.lon) annotation (Line(points={{-179,84},{-134,84},{-134,
          144}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}}));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-300,
            -210},{200,460}})),
          Documentation(info = "<html>
          <p>
          This model demonstrates one potential simulation using the models available in
          <a href=\"modelica://Buildings.Rooms.FLEXLAB.Rooms.X3A\">
          Buildings.Rooms.FLEXLAB.Rooms.X3A</a>. This example simulates test cell
          X3A when it is conditioned with a radiant slab. This example model includes
          heat transfer between the test cell, the outdoor environment, the radiant slab
          conditioning the test cell, the connected electrical room and closet, and the
          neighboring test cells.
          </p>
          <p>
          The connections between the test cell and the external models are described in the following table.
          Only models not included in the X3A package are included. For documentation describing the connections
          between X3A models see <a href=\"modelica://Buildings.Rooms.FLEXLAB.Rooms.X3A\">
          Buildings.Rooms.FLEXLAB.Rooms.X3A</a>.
          </p>
          <table border =\"1\" summary = \"Summary of connections between test cell and other models\">
          <tr>
          <th>External model name</th>
          <th>External model significance</th>
          <th>External model port</th>
          <th>X3A port</th>
          </tr>
          <tr>
          <td>weaDat</td>
          <td>Outdoor weather</td>
          <td>weaDat.weaBus</td>
          <td>X3A.weaBus</td>
          </tr>
          <tr>
          <td>TNei</td>
          <td>Neighboring test cells (X2B and X3B)</td>
          <td>X2B: X3A.preTem2.port[1]<br/>
          X3B: X3A.preTem2.port[2]</td>
          <td>X2B: X3A.surf_conBou[1]<br/>
          X3B: X3A.surf_conBou[2]</td>
          </tr>
          <tr>
          <td>sla4A1</td>
          <td>Radiant slab serving the north portion of X3A</td>
          <td>sla4A1.surf_a</td>
          <td>X3A.surf_surBou[1]</td>
          </tr>
          <tr>
          <td>sla4A2</td>
          <td>Radiant slab serving the north-central portion of X3A</td>
          <td>sla4A2.surf_a</td>
          <td>X3A.surf_surBou[2]</td>
          </tr>
          <tr>
          <td>sla4A3</td>
          <td>Radiant slab serving the south-central portion of X3A</td>
          <td>sla4A3.surf_a</td>
          <td>X3A.surf_surBou[3]</td>
          </tr>
          <tr>
          <td>sla4A4</td>
          <td>Radiant slab serving the south portion of X3A</td>
          <td>sla4A4.surf_a</td>
          <td>X3A.surf_surBou[4]</td>
          </tr>
          <tr>
          <td>shaPos</td>
          <td>Table describing the position of the window shade</td>
          <td>shaPos.y[1]</td>
          <td>X3A.uSha</td>
          </tr>
          <tr>
          <td>intGai</td>
          <td>Table specifying the internal gains in the space</td>
          <td>intGai[1,2,3]</td>
          <td>X3A.qGai_flow[1,2,3]</td>
          </tr>
          <tr>
          <td>airIn</td>
          <td>Prescribed airflow describing service air from the AHU</td>
          <td>airIn.ports[1]</td>
          <td>X3A.ports[1]</td>
          </tr>
          <tr>
          <td>airOut</td>
          <td>Outlet for ventilation air flow</td>
          <td>airOut.ports[1]</td>
          <td>X3A.ports[1]</td>
          </tr>
          </table>
          <p>
          The connections between the closet and external models are described in the following table.
          Only connections to models not included in the X3A package are described.
          </p>
          <table border=\"1\" summary = \"Summary of connections to the closet model\">
          <tr>
          <th>External model name</th>
          <th>External model significance</th>
          <th>External model port</th>
          <th>clo port</th>
          </tr>
          <tr>
          <td>intGaiClo</td>
          <td>Table specifying the internal gains in the closet</td>
          <td>intGaiClo[1,2,3]</td>
          <td>clo.qGai_flow[1,2,3]</td>
          </tr>
          <tr>
          <td>airInClo</td>
          <td>Prescribed airflow describing service air from the AHU</td>
          <td>airInClo.ports[1]</td>
          <td>clo.ports[1]</td>
          </tr>
          <tr>
          <td>airOutClo</td>
          <td>Outlet for ventilation air flow</td>
          <td>airOutClo.ports[1]</td>
          <td>clo.ports[1]</td>
          </tr>
          <tr>
          <td>preT</td>
          <td>Prescribed temperature describing the ground temperature</td>
          <td>preT.port</td>
          <td>clo.surf_conBou[3]</td>
          </tr>
          </table>
          <p>
          The connections between the electrical room and external models are described in the following
          table. Only connections to models not included in the X3A package are described.
          </p>
          <table border=\"1\" summary = \"Summary of connections to the electrical room model\">
          <tr>
          <th>External model name</th>
          <th>External model significance</th>
          <th>External model port</th>
          <th>ele port</th>
          </tr>
          <tr>
          <td>intGaiEle</td>
          <td>Table specifying the internal gains in the electrical room</td>
          <td>intGaiEle[1,2,3]</td>
          <td>ele.qGai_flow[1,2,3]</td>
          </tr>
          <tr>
          <td>airInEle</td>
          <td>Prescribed airflow describing service air from the AHU</td>
          <td>airInEle.ports[1]</td>
          <td>ele.ports[1]</td>
          </tr>
          <tr>
          <td>airOutEle</td>
          <td>Outlet for ventilation air flow</td>
          <td>airOutEle.ports[1]</td>
          <td>ele.ports[1]</td>
          </tr>
          <tr>
          <td>preT</td>
          <td>Prescribed temperature describing the ground temperature</td>
          <td>preT.port</td>
          <td>ele.surf_conBou[1]</td>
          </tr>
          </table>
          <p>
          The radiant slab is modeled using an instance of
          <a href=\"modelica://Buildings.Fluid.HeatExchangers.RadiantSlabs.SingleCircuitSlab\">
          Buildings.Fluid.HeatExchangers.RadiantSlabs.SingleCircuitSlab</a>. All of the inputs
          used to define the radiant slab are taken from the architectural drawings. The following
          table describes the connections between models used in the radiant slab. The connection
          to X3A is not included because it was previously described.
          </p>
          <table border=\"1\" summary = \"Summary of connections to the radiant slab model\">
          <tr>
          <th>External model name</th>
          <th>External model significance</th>
          <th>External model port</th>
          <th>Radiant slab port</th>
          </tr>
          <tr>
          <td>watIn</td>
          <td>Inlet for service fluid flow. Currently connects to a prescribed flow described
          in a table</td>
          <td>watIn.ports[1]</td>
          <td>sla.port_a</td>
          </tr>
          <tr>
          <td>preT</td>
          <td>Ground temperature beneath the radiant slab construction. Currently connects to
          a prescribed temperature defined in a table</td>
          <td>preT.port</td>
          <td>sla.surf_b</td>
          </tr>
          <tr>
          <td>watOut</td>
          <td>Outlet for service fluid flow</td>
          <td>watOut.ports[1]</td>
          <td>sla.port_b</td>
          </tr>
          </table>
          <p>
          The model only simulates the space conditions, the effects of the radiant slab, and the
          heat transfer between the rooms. The air handling unit, chilled water plant, shade control,
          internal gains, and ground temperature are all modeled by reading data from tables.
          Currently the ventilation air is read from an external data file, via the model
          <code>airCon</code>, while the others use tables described in the data reader model. The table
          below shows the name of data input files in the model, what physical phenomena the data file
          describes, the physical quantity of each data file output, and the source of the data.
          </p>
          <table border =\"1\" summary = \"Description of data table models\">
          <tr>
          <th>Model name</th>
          <th>Quantity described</th>
          <th>Data source</th>
          <th>y[1] significance</th>
          <th>y[2] significance</th>
          <th>y[3] significance</th>
          <th>y[4] significance</th>
          </tr>
          <tr>
          <td>shaPos</td>
          <td>Position of the shade</td>
          <td>Table in model</td>
          <td>Position of the shade</td>
          </tr>
          <tr>
          <td>intGai</td>
          <td>Internal gains</td>
          <td>Table in model</td>
          <td>Radiant heat</td>
          <td>Convective heat</td>
          <td>Latent heat</td>
          <td></td>
          </tr>
          <tr>
          <td>airCon</td>
          <td>Ventilation air from air handling unit</td>
          <td>External text file</td>
          <td>Mass flow rate</td>
          <td></td>
          <td></td>
          <td>Temperature</td>
          <td></td>
          </tr>
          <tr>
          <td>watCon</td>
          <td>Conditioning water from central plant</td>
          <td>Table in model</td>
          <td>Mass flow rate</td>
          <td>Temperature</td>
          <td></td>
          </tr>
          <tr>
          <td>TGro</td>
          <td>Ground temperature</td>
          <td>Table in model</td>
          <td>Temperature</td>
          <td></td>
          <td></td>
          </tr>
          <tr>
          <td>intGaiClo</td>
          <td>Internal gains for the closet</td>
          <td>Table in model</td>
          <td>Radiant heat</td>
          <td>Convective heat</td>
          <td>Latent heat</td>
          </tr>
          <tr>
          <td>intGaiEle</td>
          <td>Internal gains for the electrical room</td>
          <td>Table in model</td>
          <td>Radiant heat</td>
          <td>Convective heat</td>
          <td>Latent heat</td>
          </tr>
          <tr>
          <td>airConEle</td>
          <td>Ventilation air from AHU in the electrical room</td>
          <td>External text file</td>
          <td></td>
          <td></td>
          <td>Mass flow rate</td>
          <td>Temperature</td>
          </tr>
          <tr>
          <td>airConClo</td>
          <td>Ventilation air from AHU in closet</td>
          <td>External text file</td>
          <td></td>
          <td>Mass flow rate</td>
          <td></td>
          <td>Temperature</td>
          </tr>
          <tr>
          <td>TNei</td>
          <td>Temperature of the neighboring cells</td>
          <td>Table in model</td>
          <td>X2B</td>
          <td>X3B</td>
          </tr>
          </table>
          <p>
          In the above table blank entries either show that there is no data to describe, or that the data
          is describing a quantity for a separate model. Two examples are:
          <ul>
          <li>The table for shaPos only contains data for shade position. Because it only has a y[1] value
          the remaining columns in the table are left blank.</li>
          <li> airCon, airConClo, and airConEle all share an external data file. They all use the same
          temperature data, located in y[4] of the external data file. The three room models use different
          air mass flow rates. airCon uses the flow rate from y[1] in the data file, airConClo uses the
          flow rate from y[2], and airConEle uses the flow rate from y[3]. Thus, the other entries
          for each row in the table are left blank because the data is innapropriate for that particular
          model.</li>
          </ul>
          <p>
          The ventilation air flow rates used during occupied hours in this example were calculated using
          the assumption of 4 air changes per hour (ACH). It is assumed that there is
          no ventilation flow during unoccupied hours.
          </p>
          </html>",
          revisions = "<html>
          <ul>
          <li>
          December 22, 2014 by Michael Wetter:<br/>
          Removed <code>Modelica.Fluid.System</code>
          to address issue
          <a href=\"https://github.com/lbl-srg/modelica-buildings/issues/311\">#311</a>.
          </li>
          <li>September 2, 2014, by Michael Wetter:<br/>
          Corrected wrong pipe diameter.
          </li>
          <li>June 30, 2014, by Michael Wetter:<br/>
          Specified equations to be used to compute the initial conditions.</li>
          <li>October 11, 2013, by Michael Wetter:<br/>
          Added missing <code>parameter</code> keyword in the declaration of the data record.</li>
          <li>Sep 16, 2013 by Peter Grant:<br/>
          Added connections to include floor models in Closet and Electrical.</li>
          <li>Jun 10, 2013 by Peter Grant:<br/>
          First implementation.</li>
          </ul>
          </html>"),
     __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Rooms/FLEXLAB/Rooms/Examples/X3AWithRadiantFloor.mos"
        "Simulate and Plot"),
     experiment(StopTime=864000.0),
    uses(
      Modelica(version="3.2.1"),
      Buildings(version="3.0.0"),
      ModelicaServices(version="1.2")));
end X3AWithRadiantFloor;
