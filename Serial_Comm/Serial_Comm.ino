void setup()
{
	Serial.begin(9600);
}
char command_char;
char command[20];
int Spidey_Sense()
{

}
int get_command(char* list)
{
	int index = 0;
	while (Serial.available()>0)
	{
		if (index < 19)
		{
			command_char = Serial.read();
			command[index] = command_char;
			index++;
			command[index] = '\0';
		}
	}
	if (strcmp(command, list) == 0)
	{
		for (int i = 0; i<19; i++)
		{
			command[i] = 0;
		}
		index = 0;
		return(0);
	}
	else
	{
		return(1);
	}
}

void loop()
{

	if (get_command("spideysense") == 0)
	{
		Serial.write(Spidey_Sense());
	}

}
