within Buildings.DistrictEnergySystem.Loads.Examples;
model CouplingTimeSeriesMult2Loads
  "Example illustrating the coupling of a time series building model to a fluid loop"
  import Buildings;
  extends Modelica.Icons.Example;
  package Medium = Buildings.Media.Water "Fluid in the pipes";
  Buildings.BoundaryConditions.WeatherData.ReaderTMY3
                                            weaDat(
    calTSky=Buildings.BoundaryConditions.Types.SkyTemperatureCalculation.HorizontalRadiation,
    computeWetBulbTemperature=false,
    filNam=Modelica.Utilities.Files.loadResource(
        "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
    "Weather data reader"
    annotation (Placement(transformation(extent={{110,-30},{90,-10}})));

  Buildings.DistrictEnergySystem.Loads.Examples.BaseClasses.TimeSeriesBuilding2Loads
    bui(Q_flowHea_nominal={500,1000})
    annotation (Placement(transformation(extent={{38,-40},{60,-20}})));
  Buildings.Fluid.Sources.MassFlowSource_T supHea(
    use_m_flow_in=true,
    redeclare package Medium = Medium,
    T=couHea.T_a_nominal,
    nPorts=1) "Supply for heating water"          annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-48,30})));
  Buildings.Fluid.Sources.FixedBoundary sinHea(
    redeclare package Medium = Medium,
    T=couHea.T_a_nominal,
    p=300000,
    nPorts=1) "Sink for heating water"
                                      annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={100,30})));
  Buildings.Fluid.Sources.FixedBoundary sinCoo(
    redeclare package Medium = Medium,
    T=couCoo.T_a_nominal,
    p=300000,
    nPorts=1) "Sink for chilled water" annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={100,-90})));
  Buildings.Fluid.Sources.MassFlowSource_T supCoo(
    use_m_flow_in=true,
    redeclare package Medium = Medium,
    T=couCoo.T_a_nominal,
    nPorts=1)             "Supply for chilled water" annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-44,-90})));
  Buildings.DistrictEnergySystem.Loads.BaseClasses.HeatingOrCoolingMult  couHea(
    redeclare package Medium = Medium,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    nLoa=2,
    TLoa_nominal={20 + 273.15,25 + 273.15},
    T_a_nominal=318.15,
    T_b_nominal=313.15,
    Q_flowLoa_nominal={0.6,0.9} .* bui.Q_flowHea_nominal)
                                             annotation (Placement(transformation(extent={{-20,36},{0,20}})));
  Buildings.DistrictEnergySystem.Loads.BaseClasses.HeatingOrCoolingMult
                                                                    couCoo(
    redeclare package Medium = Medium,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    TLoa_nominal=fill(24 + 273.15, size(bui.Q_flowCoo_nominal, 1)),
    nLoa=1,
    reverseAction=true,
    T_a_nominal=280.15,
    T_b_nominal=285.15,
    Q_flowLoa_nominal=0.4*bui.Q_flowCoo_nominal)
                         annotation (Placement(transformation(extent={{-20,-96},{0,-80}})));
equation
  connect(weaDat.weaBus, bui.weaBus) annotation (Line(
      points={{90,-20},{49.11,-20}},
      color={255,204,51},
      thickness=0.5));

  connect(couCoo.port_b, sinCoo.ports[1]) annotation (Line(points={{0,-90},{90,-90}}, color={0,127,255}));
  connect(supCoo.ports[1], couCoo.port_a)
    annotation (Line(points={{-34,-90},{-20,-90}},                     color={0,127,255}));
  connect(couHea.port_b, sinHea.ports[1]) annotation (Line(points={{0,30},{90,30}}, color={0,127,255}));
  connect(supHea.ports[1], couHea.port_a) annotation (Line(points={{-38,30},{-20,30}}, color={0,127,255}));
  connect(bui.Q_flowHeaReq, couHea.Q_flowLoaReq)
    annotation (Line(points={{61.1,-27},{68,-27},{68,0},{-30,0},{-30,22},{-22,22}}, color={0,0,127}));
  connect(couHea.heaPorLoa, bui.heaPorHea) annotation (Line(points={{-10,20},{-10,-23},{38,-23}}, color={191,0,0}));
  connect(bui.heaPorCoo, couCoo.heaPorLoa) annotation (Line(points={{38,-37},{-10,-37},{-10,-80}}, color={191,0,0}));
  connect(couCoo.m_flowReq, supCoo.m_flow_in)
    annotation (Line(points={{1,-82},{20,-82},{20,-120},{-80,-120},{-80,-82},{-56,-82}}, color={0,0,127}));
  connect(couHea.m_flowReq, supHea.m_flow_in)
    annotation (Line(points={{1,22},{20,22},{20,60},{-80,60},{-80,38},{-60,38}}, color={0,0,127}));
  connect(bui.Q_flowCooReq, couCoo.Q_flowLoaReq)
    annotation (Line(points={{61.1,-33},{68,-33},{68,-60},{-30,-60},{-30,-82},{-22,-82}}, color={0,0,127}));
  annotation (Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-140},{120,80}})),
    __Dymola_Commands(file=
          "Resources/Scripts/Dymola/DistrictEnergySystem/Loads/Examples/CouplingTimeSeries.mos"
        "Simulate and plot"));
end CouplingTimeSeriesMult2Loads;
