within Buildings.Electrical.Interfaces;
model PartialConversion
  "Partial model representing a generic two port system for conversion"
  extends Buildings.Electrical.Interfaces.PartialTwoPort;
  Modelica.SIunits.Voltage v_p "Voltage drop between the two positive pins";
  Modelica.SIunits.Voltage v_n "Voltage drop between the two negative pins";
  Modelica.SIunits.Current i_p "Current flowing through the positive pins";
  Modelica.SIunits.Current i_n "Current flowing through the negative pins";
equation

  i_p = PhaseSystem_p.systemCurrent(terminal_p.i);
  i_n = PhaseSystem_n.systemCurrent(terminal_n.i);

  v_p = PhaseSystem_p.systemVoltage(terminal_p.v);
  v_n = PhaseSystem_n.systemVoltage(terminal_n.v);

  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}), graphics), Documentation(revisions="<html>
<ul>
<li>
May 15, 2014, by Marco Bonvini:<br/>
Created documentation.
</li>
<li>
October 31, 2013, by Marco Bonvini:<br/>
Model included into the Buildings library.
</li>
</ul>
</html>", info="<html>
<p>
This partial model extends the base <a href=\"Buildings.Electrical.Interfaces.PartialTwoPort\">Buildings.Electrical.Interfaces.PartialTwoPort</a>
model and includes variables like <code>v_p</code> and <code>i_p</code> that represents the voltage and the 
current at the <code>terminal_p</code>. These variable are used in conversion models like transformers or AC/DC converters.
</p>
</html>"));
end PartialConversion;
