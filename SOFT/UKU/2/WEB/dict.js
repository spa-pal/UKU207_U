var dict = [
	["Uмакс","В","INT","INCDEC",2000,2500,"Максимальное выходное напряжение"],
	["Uмин","В","INT","INCDEC",1500,2000,"Минимальное выходное напряжение"],
	["Uвых","В","INT/100","INCDEC",1500,2000,"Требуемое выходное напряжение"],
	["Местоположение","","STRING","INCDEC","","","Максимальное выходное напряжение"]
];
var bps_status_string = [
	"",
	"работает",
	"в резерве",
	"работает, сильный нагрев",
	"заблокирован извне",
	"работает АПВ",
	"АВАРИЯ!!! Перегрев",
	"АВАРИЯ!!! Занижено выходное напряжение",
	"АВАРИЯ!!! Завышено выходное напряжение",
	"АВАРИЯ!!! Потеряна связь",
	"Отсутствует первичное питание"
];