#include "efile.h"
#include "edisk.h"
#include "inc/tm4c123gh6pm.h"
#include "UART.h"
int StreamToFile=0;  // 0=UART, 1=stream to file
int File_init = 0;   // 0=not init, 1=init
#define DIREC_ENTRY_NUM 32
#define DIREC_PER_ENTRY 16
#define FAT_PER_ENTRY 2
#define BLOCK_SIZE 512
#define BLOCK_NUM 2048
#define DIREC_NAME_SIZE 8

BYTE Directory_Block[DIREC_ENTRY_NUM][DIREC_PER_ENTRY] = {0,}; //Directory Block, per entry is 16 bytes
WORD FAT[BLOCK_NUM] = {0,}; //FAT, per entry is 2 bytes, it needs 4096/512 = 8 blocks
BYTE One_Block[BLOCK_SIZE] = {0,};
BYTE Write_Block[BLOCK_SIZE] = {0,};
BYTE Read_Block[BLOCK_SIZE] = {0,};
int Num_File = 0;
int WOpen = 0;
int ROpen = 0;
int RPos = 0;
int First_Block_Size = 0;
int Last_Block_Size = 0;
int block_number_in_file = 1;
int last_block;
int first_block;
int open_file_index;
int read_block_num;
long file_size;
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
//---------- eFile_Init-----------------
// Activate the file system, without formating
// Input: none
// Output: 0 if successful and 1 on failure (already initialized)
// since this program initializes the disk, it must run with 
//    the disk periodic task operating
int eFile_Init(void){ // initialize file system
	if(File_init) return 1;
	else {
		File_init = 1;
		return eDisk_Init(0);  // initialize disk
	}
}

//---------- eFile_Format-----------------
// Erase all files, create blank directory, initialize free space manager
// Input: none
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Format(void){ // erase disk, add format
	int i,j;
	long st;
	st = StartCritical();
	Num_File = 0;
	//write direc
	for(i = 0; i < DIREC_ENTRY_NUM - 1; i++){
		for(j = 0; j < DIREC_PER_ENTRY; j++){
			Directory_Block[i][j] = 0;
		}
	}
	Directory_Block[DIREC_ENTRY_NUM-1][DIREC_NAME_SIZE] = 9;//first 8 types for name space, 9 is the start block
	if(eDisk_WriteBlock((BYTE*)Directory_Block,0)){
			EndCritical(st);
			return 1;
	}
	//write FAT
	for(i = 9; i < BLOCK_NUM - 1; i++){
			FAT[i] = i+1;
	}
	FAT[BLOCK_NUM-1] = 0;//end of free block
	if(eDisk_Write ( 0, (BYTE*)FAT, 1,8)){
			EndCritical(st);
			return 1;
	};
	
	//write disk
	for(i = 0; i < BLOCK_SIZE; i++){
			One_Block[i] = 0;
	}
	for(i = 9; i < BLOCK_NUM; i++){
		if(eDisk_WriteBlock (One_Block,i)) {
			EndCritical(st);
			return 1;
		}
	}
	EndCritical(st);
	return 0;
}

//find the file in the directory according to the name
int eFile_find_exist(char name[]){
	int k,i;
	char* file;
	for( k = 0; k < 31; k++){
		file = (char*)&Directory_Block[k][0];
		for(i = 0; (name[i]!='\0')&&(*file != '\0'); i++){
			if(*file != name[i]) break;
			file++;
		}
		if((name[i]=='\0')&&(*file=='\0')){ return k; }
	}
	return 32;
}

//find a empty entry in the directory
int eFile_find_entry(){
	int k;
	//find the entry for the file k
	for(k = 0 ; k<31; k++){
		if((Directory_Block[k][8] == 0)&&(Directory_Block[k][9]==0))
			return k;
	}
	return 32;
}
//write the name to the directory entry
void eFile_write_name(int index, char name[]){
	char* p = name; 
	int i = 0;
	while(*p!='\0'){
		Directory_Block[index][i] = *p;
		p++;
		i++;
	}
	Directory_Block[index][i] = '\0';
}

//write the begin block to the directory, update the free block
void eFile_write_begin(int direc_entry){
	 WORD first_free = Directory_Block[31][8] + Directory_Block[31][9]*256;
	 WORD next_free = FAT[first_free];
	 Directory_Block[31][8] = next_free%256;
	 Directory_Block[31][9] = next_free/256;
	 FAT[first_free] = 0;
	 Directory_Block[direc_entry][8] = first_free%256; 
	 Directory_Block[direc_entry][9] = first_free/256;
}

//update directory and FAT in the dict
int eFile_write2dict(){
	 if(eDisk_WriteBlock((BYTE*)Directory_Block,0)) return 1;
	 if(eDisk_Write(0,(BYTE*)FAT,1,8)) return 1;
	 return 0;
}

//find the last block of a file
int eFile_find_last(int direc_entry){
	 WORD first_block = Directory_Block[direc_entry][8]+Directory_Block[direc_entry][9]*256;
	 WORD next_block = first_block;
	 //block_number_in_file = 1;
	 while(FAT[next_block]!=0){
		 next_block = FAT[next_block];
		 //block_number_in_file++;
	 }
	 return next_block;
}
//find new free block as the last block, return the last block index
int eFile_add_last(int last){
	WORD first_free = Directory_Block[31][8] + Directory_Block[31][9]*256;
	WORD next_free = FAT[first_free];
	if(!first_free) return 0;
	Directory_Block[31][8] = next_free%256;
	Directory_Block[31][9] = next_free/256;
	FAT[first_free] = 0;
	FAT[last] = first_free;
	return first_free;
}
//find the first block of a file
int eFile_find_first(int direc_entry){
	WORD first_block = Directory_Block[direc_entry][8]+Directory_Block[direc_entry][9]*256;
	return first_block;
}
//---------- eFile_Create-----------------
// Create a new, empty file with one allocated block
// Input: file name is an ASCII string up to seven characters 
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Create( char name[]){  // create new file, make it empty 
	long st;
	char * p = name;
	int k = 0;
	int index;
	st = StartCritical();
	//check name
	while(*p != '\0'){
		k++;  p++;
	}
	if(k>7){
		EndCritical(st);
		return 1;
	}
	//check num of files
	if(Num_File == 31){
		EndCritical(st);
		return 1;
	}
	//check free block
	if((Directory_Block[31][8] == 0)&&(Directory_Block[31][9]==0)){
		EndCritical(st);
		return 1;
	}
	//check file is existed
	if(eFile_find_exist(name)!=32){
		EndCritical(st);
		return 0; //ignore error?
	}
	//add to the file system
	index = eFile_find_entry();
	eFile_write_name(index,name);
	eFile_write_begin(index);
	if(eFile_write2dict()){
		EndCritical(st);
		return 1;
	}
	Num_File++;
	EndCritical(st);
	return 0;
}


//---------- eFile_WOpen-----------------
// Open the file, read into RAM last block
// Input: file name is a single ASCII letter
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_WOpen(char name[]){     // open a file for writing 
	long st;
	st = StartCritical();
	if(WOpen){
		EndCritical(st);
		return 1;
	}
	open_file_index = eFile_find_exist(name);
	if(open_file_index == 32){
		EndCritical(st);
		return 1;
	}
	last_block = eFile_find_last(open_file_index);
	if(eDisk_ReadBlock(Write_Block,last_block)){
		EndCritical(st);
		return 1;
	}
	file_size = Directory_Block[open_file_index][12] + Directory_Block[open_file_index][13]*256;
	if(file_size%BLOCK_SIZE)Last_Block_Size = file_size%BLOCK_SIZE;
	else if(file_size!=0)Last_Block_Size= BLOCK_SIZE;
	WOpen = 1;
	EndCritical(st);
	return 0;
}

//---------- eFile_Write-----------------
// save at end of the open file
// Input: data to be saved
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Write( char data){
	long st;
	int k;
	st = StartCritical();
	if(!WOpen){
		EndCritical(st);
		return 1;
	}
	if(Last_Block_Size + 1< 512){
		Write_Block[Last_Block_Size] = data;
		Last_Block_Size++;
	}else{
		if(Last_Block_Size + 1 == 512){//full after add one data
			Write_Block[Last_Block_Size] = data;
			if(eDisk_WriteBlock(Write_Block,last_block)){
				EndCritical(st);
				return 1;
			}
			Last_Block_Size = 0;
		}
		//update last_block and Write_Block
		//new write block
			for(k = 0; k < BLOCK_SIZE; k++){
				Write_Block[k] = 0;
			}
			if(Last_Block_Size == 512){
				Write_Block[0] = data;
				Last_Block_Size = 1;
			}
			last_block = eFile_add_last(last_block);
			if(last_block == 0){//disc full
				EndCritical(st);
				return 1;
			}
	}
	//update the size in the directory
	file_size++;
	EndCritical(st);
	return 0;
}

//---------- eFile_Close-----------------
// Deactivate the file system
// Input: none
// Output: 0 if successful and 1 on failure (not currently open)
int eFile_Close(void){
	long st;
	st = StartCritical();
	if((WOpen == 0)&&(ROpen == 0)){
		EndCritical(st);
		return 1;
	}
	WOpen = 0;
	ROpen = 0;
	EndCritical(st);
	return 0;
}


//---------- eFile_WClose-----------------
// close the file, left disk in a state power can be removed
// Input: none
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_WClose(void){ // close the file for writing
	long st;
	st = StartCritical();
	if(WOpen == 0){
		EndCritical(st);
		return 1;
	}
	if(Last_Block_Size != 0){
		if(eDisk_WriteBlock(Write_Block,last_block)){
			EndCritical(st);
			return 1;
		}
	}
	WOpen = 0;
	//update the size in the directory
	Directory_Block[open_file_index][12] = file_size%256;
	Directory_Block[open_file_index][13] = file_size/256;
	eFile_write2dict();
	EndCritical(st);
	return 0;
}

//---------- eFile_ROpen-----------------
// Open the file, read first block into RAM 
// Input: file name is a single ASCII letter
// Output: 0 if successful and 1 on failure (e.g., trouble read to flash)
int eFile_ROpen( char name[]){      // open a file for reading 
	long st;
	st = StartCritical();
	if(ROpen){
		EndCritical(st);
		return 1;
	}
	open_file_index = eFile_find_exist(name);
	if(open_file_index == 32){
		EndCritical(st);
		return 1;
	}
	first_block = eFile_find_first(open_file_index);
	if(eDisk_ReadBlock(Read_Block,first_block)){
		EndCritical(st);
		return 1;
	}
	file_size = Directory_Block[open_file_index][12] + Directory_Block[open_file_index][13]*256;
	if(file_size/BLOCK_SIZE) First_Block_Size = BLOCK_SIZE;
	else First_Block_Size = file_size%BLOCK_SIZE;
	ROpen = 1;
	RPos  = 0;
	read_block_num = 1;
	EndCritical(st);
	return 0;
}
   
//---------- eFile_ReadNext-----------------
// retreive data from open file
// Input: none
// Output: return by reference data
//         0 if successful and 1 on failure (e.g., end of file)
int eFile_ReadNext( char *pt){       // get next byte 
	long st;
	st = StartCritical();
	if(!ROpen){
		EndCritical(st);
		return 1;
	}
	if(First_Block_Size==0){
		EndCritical(st);
		return 1;
	}
	//at the end of current block
	if(RPos == First_Block_Size){
		first_block = FAT[first_block];
		if(first_block == 0){//wrong
			EndCritical(st);
			return 1;
		}
		//next block of the file
		if((file_size-BLOCK_SIZE*read_block_num)/BLOCK_SIZE)First_Block_Size = BLOCK_SIZE;
		else First_Block_Size = (file_size-BLOCK_SIZE*read_block_num)%BLOCK_SIZE;
		read_block_num ++;
		RPos = 0;
		if(eDisk_ReadBlock(Read_Block,first_block)){
			EndCritical(st);
			return 1;
		}
	}
	*pt = Read_Block[RPos];
	RPos++;
	EndCritical(st);
	return 0;
}
                              
//---------- eFile_RClose-----------------
// close the reading file
// Input: none
// Output: 0 if successful and 1 on failure (e.g., wasn't open)
int eFile_RClose(void){ // close the file for writing
	long st;
	st = StartCritical();
	if(ROpen == 0){
		EndCritical(st);
		return 1;
	}
	ROpen = 0;
	EndCritical(st);
	return 0;
}

//---------- eFile_Directory-----------------
// Display the directory with filenames and sizes
// Input: pointer to a function that outputs ASCII characters to display
// Output: characters returned by reference
//         0 if successful and 1 on failure (e.g., trouble reading from flash)
int eFile_Directory(void(*fp)(char)){
	long st;
	int k, i;
	st = StartCritical();
	for(k = 0; k < 31; k++){
		//name
		if((Directory_Block[k][8]!=0)||(Directory_Block[k][9]!=0)){//if this entry is not empty
			i = 0;
			while(Directory_Block[k][i]!='\0'){
				fp(Directory_Block[k][i]);
				i++;
			}
			UART_OutString(":");
			file_size = Directory_Block[k][12] + Directory_Block[k][13]*256;
			//fp(file_size);
			UART_OutUDec(file_size);
			UART_OutString(" bytes\r\n");
		}
	}
	EndCritical(st);
	return 0;
}  

//---------- eFile_Delete-----------------
// delete this file
// Input: file name is a single ASCII letter
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Delete( char name[]){  // remove this file 
	long st;
	int k;
	st = StartCritical();
	open_file_index = eFile_find_exist(name);
	if(open_file_index == 32){
		EndCritical(st);
		return 1;
	}
	//modify the diretory
	first_block = eFile_find_first(open_file_index);
	for(k = 0; k < 16; k++){
		Directory_Block[open_file_index][k] = 0;
	}
	//move to free block
	last_block = eFile_find_last(31);
	FAT[last_block] = first_block;
	//update dict
	for(k = 0; k < BLOCK_SIZE; k++){
		One_Block[k] = 0;
	}
	do{
		if(eDisk_WriteBlock(One_Block,first_block)){
			EndCritical(st);
			return 1;
		}
		first_block = FAT[first_block];
	}while(first_block!=0);
	Num_File--;
	//update directory and FAT
	eFile_write2dict();
	EndCritical(st);
	return 0;
}

/*
int eFile_Delete( char name[]) {
DRESULT r1,r2,r3;
int i;
int index;
int tmpFAT;
int	index_FAT = 0;
long status;
status = StartCritical();
for (i=0; i<BLOCK_SIZE; i++) {
Write_Block[i] = 0;
}

index=eFile_find_exit(name);
if(index == 32){
EndCritical(status);	
return 1;
}
else {
index_FAT = Directory_Block[index][8] + Directory_Block[index][9] >> 8;
for (i=0; i<16; i++)
Directory_Block[index][i] = 0;

while(index_FAT!=0){
r3=eDisk_Write(0, (BYTE*)One_Block, index_FAT, 1);
if(result3!=0){
EndCritical(status);
return 1;
}
tmpFAT=FAT[index_FAT];
FAT[index_FAT]=0;
index_FAT=tmpFAT;
}

r1 = eDisk_WriteBlock ((BYTE*)Directory_Block, 0);    
r2 = eDisk_WriteBlock ((BYTE*)FAT, 1); 

if(r1==0&&r2==0) {
EndCritical(status);
return 0;
}
else {
EndCritical(status);
return 1;
}
}
EndCritical(status);	
return 1;
}
*/
//---------- eFile_RedirectToFile-----------------
// open a file for writing 
// Input: file name is a single ASCII letter
// stream printf data into file
// Output: 0 if successful and 1 on failure (e.g., trouble read/write to flash)
int eFile_RedirectToFile(char *name){
 eFile_Create(name); // ignore error if file already exists
 if(eFile_WOpen(name)) return 1; // cannot open file
 StreamToFile = 1;
 return 0;
}

//---------- eFile_EndRedirectToFile-----------------
// close the previously open file
// redirect printf data back to UART
// Output: 0 if successful and 1 on failure (e.g., wasn't open)
int eFile_EndRedirectToFile(void){
 StreamToFile = 0;
 if(eFile_WClose()) return 1;  // cannot close file
 return 0;
}



