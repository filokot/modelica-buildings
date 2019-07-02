within Buildings.DistrictEnergySystem.Loads.Examples.BaseClasses;
model GeojsonExportBuilding "Building model of type RC based on Urbanopt GeoJSON export"
  import Buildings;
  extends Buildings.DistrictEnergySystem.Loads.BaseClasses.PartialBuilding(
    final heaLoaTyp=fill(Buildings.DistrictEnergySystem.Loads.Types.ModelType.HeatPort, nHeaLoa),
    final cooLoaTyp=fill(Buildings.DistrictEnergySystem.Loads.Types.ModelType.HeatPort, nCooLoa),
    final nHeaLoa=6,
    final nCooLoa=6,
    Q_flowCoo_nominal={30000,5000,5000,5000,5000,20000},
    Q_flowHea_nominal={15000,10000,5000,8000,5000,1000});
  Buildings.DistrictEnergySystem.Loads.Examples.BaseClasses.GeojsonExport.B5a6b99ec37f4de7f94020090_Office
    b5a6b99ec37f4de7f94020090_Office annotation (Placement(transformation(extent={{-100,-20},{-80,0}})));
  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant minTSet[nHeaLoa](k=fill(20, nHeaLoa))
    "Minimum temperature setpoint" annotation (Placement(transformation(extent={{-140,120},{-120,140}})));
  Buildings.Controls.OBC.UnitConversions.From_degC from_degC1[nHeaLoa]
    annotation (Placement(transformation(extent={{-100,120},{-80,140}})));
  Buildings.Controls.OBC.CDL.Continuous.LimPID conPIDMinT[nHeaLoa](
    each yMax=1,
    each controllerType=Buildings.Controls.OBC.CDL.Types.SimpleController.PI,
    each reverseAction=false,
    each yMin=0,
    each Ti=120) "PID controller for minimum temperature"
    annotation (Placement(transformation(extent={{-60,120},{-40,140}})));
  Buildings.Controls.OBC.CDL.Continuous.Gain gai[nHeaLoa](k=Q_flowHea_nominal)
    annotation (Placement(transformation(extent={{-20,120},{0,140}})));
  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant maxTSet[nCooLoa](k=fill(24, nCooLoa))
    "Maximum temperature setpoint" annotation (Placement(transformation(extent={{-140,-140},{-120,-120}})));
  Buildings.Controls.OBC.UnitConversions.From_degC from_degC2[nCooLoa]
    annotation (Placement(transformation(extent={{-100,-140},{-80,-120}})));
  Buildings.Controls.OBC.CDL.Continuous.LimPID conPIDMax[nCooLoa](
    each controllerType=Buildings.Controls.OBC.CDL.Types.SimpleController.PI,
    each reverseAction=true,
    each yMax=1,
    each yMin=0,
    each Ti=120) "PID controller for maximum temperature"
    annotation (Placement(transformation(extent={{-60,-140},{-40,-120}})));
  Buildings.Controls.OBC.CDL.Continuous.Gain gai1[nCooLoa](k=-Q_flowCoo_nominal)
    annotation (Placement(transformation(extent={{-20,-140},{0,-120}})));
  Buildings.DistrictEnergySystem.Loads.Examples.BaseClasses.GeojsonExport.B5a6b99ec37f4de7f94020090_Floor
    b5a6b99ec37f4de7f94020090_Floor annotation (Placement(transformation(extent={{-60,-20},{-40,0}})));
  Buildings.DistrictEnergySystem.Loads.Examples.BaseClasses.GeojsonExport.B5a6b99ec37f4de7f94020090_Storage
    b5a6b99ec37f4de7f94020090_Storage annotation (Placement(transformation(extent={{-20,-20},{0,0}})));
  Buildings.DistrictEnergySystem.Loads.Examples.BaseClasses.GeojsonExport.B5a6b99ec37f4de7f94020090_Meeting
    b5a6b99ec37f4de7f94020090_Meeting annotation (Placement(transformation(extent={{20,-20},{40,0}})));
  Buildings.DistrictEnergySystem.Loads.Examples.BaseClasses.GeojsonExport.B5a6b99ec37f4de7f94020090_Restroom
    b5a6b99ec37f4de7f94020090_Restroom annotation (Placement(transformation(extent={{60,-20},{80,0}})));
  Buildings.DistrictEnergySystem.Loads.Examples.BaseClasses.GeojsonExport.B5a6b99ec37f4de7f94020090_ICT
    b5a6b99ec37f4de7f94020090_ICT annotation (Placement(transformation(extent={{100,-20},{120,0}})));
equation
  connect(heaPorHea[1], b5a6b99ec37f4de7f94020090_Office.port_a)
    annotation (Line(points={{-300,91.6667},{-200,91.6667},{-200,0},{-90,0}},
                                                                        color={191,0,0}));
  connect(heaPorCoo[1], b5a6b99ec37f4de7f94020090_Office.port_a)
    annotation (Line(points={{-300,-108.333},{-200,-108.333},{-200,0},{-90,0}},
                                                                          color={191,0,0}));
  connect(from_degC1.y,conPIDMinT. u_s) annotation (Line(points={{-79,130},{-62,130}}, color={0,0,127}));
  connect(conPIDMinT.y,gai. u) annotation (Line(points={{-39,130},{-22,130}}, color={0,0,127}));
  connect(from_degC2.y,conPIDMax. u_s) annotation (Line(points={{-79,-130},{-62,-130}}, color={0,0,127}));
  connect(maxTSet.y,from_degC2. u) annotation (Line(points={{-119,-130},{-102,-130}}, color={0,0,127}));
  connect(conPIDMax.y,gai1. u) annotation (Line(points={{-39,-130},{-22,-130}}, color={0,0,127}));
  connect(heaPorHea[2], b5a6b99ec37f4de7f94020090_Floor.port_a)
    annotation (Line(points={{-300,95},{-194,95},{-194,0},{-50,0}},   color={191,0,0}));
  connect(heaPorHea[3], b5a6b99ec37f4de7f94020090_Storage.port_a)
    annotation (Line(points={{-300,98.3333},{-188,98.3333},{-188,0},{-10,0}},  color={191,0,0}));
  connect(heaPorHea[4], b5a6b99ec37f4de7f94020090_Meeting.port_a)
    annotation (Line(points={{-300,101.667},{-182,101.667},{-182,0},{30,0}},   color={191,0,0}));
  connect(heaPorHea[5], b5a6b99ec37f4de7f94020090_Restroom.port_a)
    annotation (Line(points={{-300,105},{-176,105},{-176,0},{70,0}},   color={191,0,0}));
  connect(heaPorHea[6], b5a6b99ec37f4de7f94020090_ICT.port_a)
    annotation (Line(points={{-300,108.333},{-170,108.333},{-170,0},{110,0}}, color={191,0,0}));
  connect(heaPorCoo[2], b5a6b99ec37f4de7f94020090_Floor.port_a)
    annotation (Line(points={{-300,-105},{-194,-105},{-194,0},{-50,0}},   color={191,0,0}));
  connect(heaPorCoo[3], b5a6b99ec37f4de7f94020090_Storage.port_a)
    annotation (Line(points={{-300,-101.667},{-188,-101.667},{-188,0},{-10,0}},  color={191,0,0}));
  connect(heaPorCoo[4], b5a6b99ec37f4de7f94020090_Meeting.port_a)
    annotation (Line(points={{-300,-98.3333},{-182,-98.3333},{-182,0},{30,0}},   color={191,0,0}));
  connect(heaPorCoo[5], b5a6b99ec37f4de7f94020090_Restroom.port_a)
    annotation (Line(points={{-300,-95},{-176,-95},{-176,0},{70,0}},   color={191,0,0}));
  connect(heaPorCoo[6], b5a6b99ec37f4de7f94020090_ICT.port_a)
    annotation (Line(points={{-300,-91.6667},{-170,-91.6667},{-170,0},{110,0}}, color={191,0,0}));
  connect(minTSet.y, from_degC1.u)
    annotation (Line(points={{-119,130},{-110,130},{-110,130},{-102,130}}, color={0,0,127}));
  connect(b5a6b99ec37f4de7f94020090_Floor.TAir, conPIDMinT[2].u_m)
    annotation (Line(points={{-39,-10},{-30,-10},{-30,100},{-50,100},{-50,118}}, color={0,0,127}));
  connect(b5a6b99ec37f4de7f94020090_Meeting.TAir, conPIDMinT[4].u_m)
    annotation (Line(points={{41,-10},{50,-10},{50,100},{-50,100},{-50,118}}, color={0,0,127}));
  connect(b5a6b99ec37f4de7f94020090_Restroom.TAir, conPIDMinT[5].u_m)
    annotation (Line(points={{81,-10},{90,-10},{90,100},{-50,100},{-50,118}}, color={0,0,127}));
  connect(b5a6b99ec37f4de7f94020090_ICT.TAir, conPIDMinT[6].u_m)
    annotation (Line(points={{121,-10},{130,-10},{130,100},{-50,100},{-50,118}}, color={0,0,127}));
  connect(b5a6b99ec37f4de7f94020090_Floor.TAir, conPIDMax[2].u_m)
    annotation (Line(points={{-39,-10},{-30,-10},{-30,-160},{-50,-160},{-50,-142}}, color={0,0,127}));
  connect(b5a6b99ec37f4de7f94020090_Storage.TAir, conPIDMax[3].u_m)
    annotation (Line(points={{1,-10},{10,-10},{10,-160},{-50,-160},{-50,-142}}, color={0,0,127}));
  connect(b5a6b99ec37f4de7f94020090_Meeting.TAir, conPIDMax[4].u_m)
    annotation (Line(points={{41,-10},{50,-10},{50,-160},{-50,-160},{-50,-142}}, color={0,0,127}));
  connect(b5a6b99ec37f4de7f94020090_ICT.TAir, conPIDMax[6].u_m)
    annotation (Line(points={{121,-10},{130,-10},{130,-160},{-50,-160},{-50,-142}}, color={0,0,127}));
  connect(gai1.y, Q_flowCooReq) annotation (Line(points={{1,-130},{152,-130},{152,-100},{310,-100}}, color={0,0,127}));
  connect(gai.y, Q_flowHeaReq) annotation (Line(points={{1,130},{152,130},{152,100},{310,100}}, color={0,0,127}));
  connect(b5a6b99ec37f4de7f94020090_Office.TAir, conPIDMinT[1].u_m)
    annotation (Line(points={{-79,-10},{-70,-10},{-70,100},{-50,100},{-50,118}}, color={0,0,127}));
  connect(b5a6b99ec37f4de7f94020090_Office.TAir, conPIDMax[1].u_m)
    annotation (Line(points={{-79,-10},{-70,-10},{-70,-160},{-50,-160},{-50,-142}}, color={0,0,127}));
  connect(b5a6b99ec37f4de7f94020090_Restroom.TAir, conPIDMax[5].u_m)
    annotation (Line(points={{81,-10},{90,-10},{90,-160},{-50,-160},{-50,-142}}, color={0,0,127}));
  connect(b5a6b99ec37f4de7f94020090_Storage.TAir, conPIDMinT[3].u_m)
    annotation (Line(points={{1,-10},{10,-10},{10,100},{-50,100},{-50,118}}, color={0,0,127}));
  connect(weaBus, b5a6b99ec37f4de7f94020090_Office.weaBus) annotation (Line(
      points={{1,300},{9,300},{9,20.3398},{-100,20.3398},{-100,-10}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-3,6},{-3,6}},
      horizontalAlignment=TextAlignment.Right));
  connect(weaBus, b5a6b99ec37f4de7f94020090_Floor.weaBus) annotation (Line(
      points={{1,300},{10,300},{10,20},{-60,20},{-60,-10}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-3,-6},{-3,-6}},
      horizontalAlignment=TextAlignment.Right));
  connect(weaBus, b5a6b99ec37f4de7f94020090_Storage.weaBus) annotation (Line(
      points={{1,300},{11,300},{11,20},{-20,20},{-20,-10}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-3,6},{-3,6}},
      horizontalAlignment=TextAlignment.Right));
  connect(weaBus, b5a6b99ec37f4de7f94020090_Meeting.weaBus) annotation (Line(
      points={{1,300},{10,300},{10,20},{20,20},{20,-10}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(weaBus, b5a6b99ec37f4de7f94020090_Restroom.weaBus) annotation (Line(
      points={{1,300},{9,300},{9,19},{60,19},{60,-10}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-3,6},{-3,6}},
      horizontalAlignment=TextAlignment.Right));
  connect(weaBus, b5a6b99ec37f4de7f94020090_ICT.weaBus) annotation (Line(
      points={{1,300},{10,300},{10,20},{100,20},{100,-10}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  annotation (
  Documentation(info="
  <html>
  <p>
  This is a simplified multizone RC model resulting from the translation of a GeoJSON model specified 
  within Urbanopt UI. It is composed of 6 thermal zones corresponding to the different load patterns.
  </p>
  </html>"),
  Diagram(coordinateSystem(extent={{-300,-300},{300,300}})), Icon(
        coordinateSystem(extent={{-100,-100},{100,100}})));
end GeojsonExportBuilding;
