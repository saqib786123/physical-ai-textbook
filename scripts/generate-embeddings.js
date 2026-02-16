import fs from 'fs';
import path from 'path';
import dotenv from 'dotenv';
import { OpenAIEmbeddings } from '@langchain/openai';
import { FaissStore } from '@langchain/community/vectorstores/faiss';
import { RecursiveCharacterTextSplitter } from 'langchain/text_splitter';

dotenv.config();

const DOCS_DIR = path.join(process.cwd(), 'docs');
const OUTPUT_DIR = path.join(process.cwd(), 'static', 'vector-store');

// Ensure output directory exists
if (!fs.existsSync(OUTPUT_DIR)) {
    fs.mkdirSync(OUTPUT_DIR, { recursive: true });
}

async function getAllFiles(dirPath, arrayOfFiles = []) {
    const files = fs.readdirSync(dirPath);

    for (const file of files) {
        if (fs.statSync(dirPath + '/' + file).isDirectory()) {
            arrayOfFiles = await getAllFiles(dirPath + '/' + file, arrayOfFiles);
        } else {
            if (file.endsWith('.md') || file.endsWith('.mdx')) {
                arrayOfFiles.push(path.join(dirPath, '/', file));
            }
        }
    }

    return arrayOfFiles;
}

async function generateEmbeddings() {
    console.log('🔍 Scanning documentation...');
    const files = await getAllFiles(DOCS_DIR);
    console.log(`📚 Found ${files.length} documents.`);

    const docs = [];

    for (const file of files) {
        const content = fs.readFileSync(file, 'utf8');
        // Simple metadata extraction (title usually in frontmatter or first h1)
        const relativePath = path.relative(DOCS_DIR, file);

        docs.push({
            pageContent: content,
            metadata: { source: relativePath }
        });
    }

    console.log('✂️  Splitting documents...');
    const splitter = new RecursiveCharacterTextSplitter({
        chunkSize: 1000,
        chunkOverlap: 200,
    });

    const splitDocs = await splitter.createDocuments(
        docs.map(d => d.pageContent),
        docs.map(d => d.metadata)
    );

    console.log(`🧱 Created ${splitDocs.length} chunks.`);

    if (!process.env.OPENAI_API_KEY) {
        console.error("❌ Error: OPENAI_API_KEY is not set in .env file.");
        return;
    }

    console.log('🧠 Generating embeddings (this may take a while)...');

    const embeddings = new OpenAIEmbeddings();
    const vectorStore = await FaissStore.fromDocuments(splitDocs, embeddings);

    console.log('💾 Saving vector store...');
    await vectorStore.save(OUTPUT_DIR);
    console.log(`✅ Vector store saved to ${OUTPUT_DIR}`);
}

generateEmbeddings().catch(console.error);
