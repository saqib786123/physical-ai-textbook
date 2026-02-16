const axios = require('axios');
const fs = require('fs');
const path = require('path');
const { RecursiveCharacterTextSplitter } = require("@langchain/textsplitters");
const { OpenAIEmbeddings } = require("@langchain/openai");
const { FaissStore } = require("@langchain/community/vectorstores/faiss");
require('dotenv').config();

const DOCS_DIR = path.join(process.cwd(), 'docs');
const OUTPUT_DIR = path.join(process.cwd(), 'backend', 'vector-store');

if (!process.env.OPENAI_API_KEY) {
    console.error("❌ ERROR: OPENAI_API_KEY is not defined in .env file.");
    process.exit(1);
}

// Ensure output directory exists
if (!fs.existsSync(OUTPUT_DIR)) {
    fs.mkdirSync(OUTPUT_DIR, { recursive: true });
}

function getAllFiles(dirPath, arrayOfFiles = []) {
    files = fs.readdirSync(dirPath);

    files.forEach(function (file) {
        if (fs.statSync(dirPath + "/" + file).isDirectory()) {
            arrayOfFiles = getAllFiles(dirPath + "/" + file, arrayOfFiles);
        } else {
            if (file.endsWith('.md') || file.endsWith('.mdx')) {
                arrayOfFiles.push(path.join(dirPath, "/", file));
            }
        }
    });

    return arrayOfFiles;
}

async function run() {
    console.log('🔄 Starting RAG Indexing...');

    const files = getAllFiles(DOCS_DIR);
    console.log(`📚 Found ${files.length} markdown files.`);

    const documents = [];

    for (const file of files) {
        const text = fs.readFileSync(file, 'utf8');
        const relativePath = path.relative(DOCS_DIR, file);

        // Create base document
        documents.push({
            pageContent: text,
            metadata: { source: relativePath }
        });
    }

    const splitter = new RecursiveCharacterTextSplitter({
        chunkSize: 1000,
        chunkOverlap: 200,
    });

    const splitDocs = await splitter.createDocuments(
        documents.map(d => d.pageContent),
        documents.map(d => d.metadata)
    );

    console.log(`🧩 Split into ${splitDocs.length} chunks.`);

    console.log('🧠 Generating Embeddings & Vector Store (with rate limiting)...');
    const embeddings = new OpenAIEmbeddings();

    // Process in batches to avoid rate limits
    const BATCH_SIZE = 5;
    const DELAY_MS = 1000; // 1 second delay

    const vectorStore = await FaissStore.fromDocuments(splitDocs.slice(0, 1), embeddings); // Initialize with first doc

    for (let i = 1; i < splitDocs.length; i += BATCH_SIZE) {
        const batch = splitDocs.slice(i, i + BATCH_SIZE);
        console.log(`Processing batch ${Math.ceil(i / BATCH_SIZE) + 1}/${Math.ceil(splitDocs.length / BATCH_SIZE)}...`);

        await vectorStore.addDocuments(batch);

        // Wait to respect rate limits
        await new Promise(resolve => setTimeout(resolve, DELAY_MS));
    }

    await vectorStore.save(OUTPUT_DIR);
    console.log(`✅ Vector store saved to: ${OUTPUT_DIR}`);
}

run();
